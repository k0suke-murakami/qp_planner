#include <autoware_msgs/Waypoint.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2/utils.h>
// #include <tf/transform_datatypes.h>

//headers in Eigen
// #include <Eigen/Dense>
#include <distance_transform/distance_transform.hpp>
#include <grid_map_core/GridMap.hpp>
#include <qpOASES.hpp>
#include <qpOASES/QProblem.hpp>
// #include <qpoases_vendor/qpOASES.hpp>

#include <memory>

#include "reference_path.h"


#include "qp_planner.h"


QPPlanner::QPPlanner():
number_of_sampling_points_(200),
is_solver_initialized_(false)
{
  solver_ptr_.reset(new qpOASES::QProblemB(number_of_sampling_points_));
  solver_ptr_->setPrintLevel(qpOASES::PL_NONE);
}

QPPlanner::~QPPlanner()
{
}

double QPPlanner::calculate2DDistace(const Eigen::Vector2d& point1,
                          const Eigen::Vector2d& point2)
{
  double dx = point1(0) - point2(0);
  double dy = point1(1) - point2(1);
  double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
  return distance;
}

void QPPlanner::doPlan(
  const geometry_msgs::TransformStamped& lidar2map_tf,
  const geometry_msgs::PoseStamped& in_current_pose,
  const grid_map::GridMap& grid_map,
  const std::vector<autoware_msgs::Waypoint>& in_reference_waypoints_in_lidar,
  std::vector<autoware_msgs::Waypoint>& out_waypoints)
{
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin1 = std::chrono::high_resolution_clock::now();

  std::string layer_name = grid_map.getLayers().back();
  grid_map::Matrix grid_data = grid_map.get(layer_name);

  // grid_length y and grid_length_x respectively
  dope::Index2 size({ 200, 600 });
  dope::Grid<float, 2> f(size);
  dope::Grid<dope::SizeType, 2> indices(size);
  bool is_empty_cost = true;
  for (dope::SizeType i = 0; i < size[0]; ++i)
  {
    for (dope::SizeType j = 0; j < size[1]; ++j)
    {
      if (grid_data(i * size[1] + j) > 0.01)
      {
        f[i][j] = 0.0f;
        is_empty_cost = false;
      }
      else
      {
        f[i][j] = std::numeric_limits<float>::max();
      }
    }
  }
  // Note: this is necessary at least at the first distance transform execution
  // and every time a reset is desired; it is not, instead, when updating
  dt::DistanceTransform::initializeIndices(indices);

  
  dt::DistanceTransform::distanceTransformL2(f, f, false, 1);
  
  for (dope::SizeType i = 0; i < size[0]; ++i)
  {
    for (dope::SizeType j = 0; j < size[1]; ++j)
    {
      if (is_empty_cost)
      {
        grid_data(i * size[1] + j) = 1;
      }
      else
      {
        grid_data(i * size[1] + j) = f[i][j];
      }
    }
  }
  grid_map::GridMap clearance_map;
  clearance_map.setFrameId(grid_map.getFrameId());
  clearance_map.setGeometry(grid_map.getLength(), 
                            grid_map.getResolution(),
                            grid_map.getPosition());
  std::string clearance_layer = "CLEARANCE_LAYER";
  clearance_map.add(clearance_layer, grid_data);
  
  // 3. 現在日時を再度取得
  std::chrono::high_resolution_clock::time_point end1 = std::chrono::high_resolution_clock::now();
  // 経過時間を取得
  std::chrono::nanoseconds elapsed_time1 = std::chrono::duration_cast<std::chrono::nanoseconds>(end1 - begin1);
  std::cout <<"distance transform " <<elapsed_time1.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin2 = std::chrono::high_resolution_clock::now();

  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(const auto& point: in_reference_waypoints_in_lidar)
  {
    tmp_x.push_back(point.pose.pose.position.x);
    tmp_y.push_back(point.pose.pose.position.y);
  }
  ReferencePath reference_path(tmp_x, tmp_y, 0.2);
  
  // 経過時間を取得
  std::chrono::high_resolution_clock::time_point end2 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time2 = 
  std::chrono::duration_cast<std::chrono::nanoseconds>(end2 - begin2);
  std::cout <<"frenet transform " <<elapsed_time2.count()/(1000.0*1000.0)<< " milli sec" << std::endl;

  // 1. 現在日時を取得
  // std::chrono::high_resolution_clock::time_point begin3 = std::chrono::high_resolution_clock::now();
  
  
  //TODO: delete this chunk; unnecessary
  std::vector<double> lateral_reference_point_vec;
  std::vector<double> longitudinal_reference_point_vec;
  std::vector<double> lower_bound_vec;
  std::vector<double> upper_bound_vec;
  for(size_t i = 0; i < number_of_sampling_points_; i++)
  {
    lateral_reference_point_vec.push_back(reference_path.y_[i]);
    longitudinal_reference_point_vec.push_back(reference_path.x_[i]);
    lower_bound_vec.push_back(-100);
    upper_bound_vec.push_back(100);
  }
  
  
  
  double h_matrix[number_of_sampling_points_*number_of_sampling_points_];
  double g_matrix[number_of_sampling_points_];
  // double a_constraint_matrix[number_of_sampling_points_*number_of_sampling_points_];
  double lower_bound[number_of_sampling_points_];
  double upper_bound[number_of_sampling_points_];
  double constrain[number_of_sampling_points_];
  Eigen::MatrixXd a_constraint = Eigen::MatrixXd::Identity(number_of_sampling_points_, number_of_sampling_points_);
  
  
  Eigen::MatrixXd tmp_a1 = Eigen::MatrixXd::Identity(number_of_sampling_points_, number_of_sampling_points_);
  Eigen::MatrixXd tmp_a2(number_of_sampling_points_, number_of_sampling_points_);
  Eigen::MatrixXd tmp_a3(number_of_sampling_points_, number_of_sampling_points_);
  // Eigen::MatrixXd tmp_a_constrain(number_of_sampling_points_,number_of_sampling_points_);
  Eigen::VectorXd tmp_b1(number_of_sampling_points_);
  Eigen::VectorXd tmp_b2(number_of_sampling_points_);
  Eigen::VectorXd tmp_b3(number_of_sampling_points_);
  for (int r = 0; r < number_of_sampling_points_; ++r)
  {
    tmp_b1(r) = lateral_reference_point_vec[r];
    tmp_b2(r) = 0;
    tmp_b3(r) = 0;
    for (int c = 0; c < number_of_sampling_points_; ++c)
    {
      if(c==r && c != number_of_sampling_points_-1)
      {
        tmp_a2(r, c) = 1;
      }
      else if(c - r == 1)
      {
        tmp_a2(r, c) = -1;
      }
      else
      {
        tmp_a2(r, c) = 0;
      }
      
      if(r == number_of_sampling_points_-1 || r == number_of_sampling_points_ - 2)
      {
        tmp_a3(r, c) = 0;
      }
      else if(c==r)
      {
        tmp_a3(r, c) = 1;
      }
      else if(c - r == 1)
      {
        tmp_a3(r, c) = -2;
      }
      else if(c - r == 2)
      {
        tmp_a3(r, c) = 1;
      }
      else
      {
        tmp_a3(r, c) = 0;
      }    
    }
  }
  
  double w1 = 0.00001;
  double w3 = 1.0;
  // double w1 = 1.0;
  // double w3 = 1.0;
  Eigen::MatrixXd tmp_a = w1*tmp_a1.transpose()*tmp_a1 + 
                          // tmp_a2.transpose()*tmp_a2 +
                          w3*tmp_a3.transpose()*tmp_a3;
  Eigen::MatrixXd tmp_b = -1*(w1*tmp_b1.transpose()*tmp_a1 +
                              // tmp_b2.transpose()*tmp_a2 +
                              w3*tmp_b3.transpose()*tmp_a3);
  
  // Eigen::MatrixXd tmp_a_constrain = 
  //                         tmp_a2.transpose()*tmp_a2;
  
  
  
  int index = 0;

  for (int r = 0; r < number_of_sampling_points_; ++r)
  {
    for (int c = 0; c < number_of_sampling_points_; ++c)
    {
      h_matrix[index] = tmp_a(r, c);
      // h_matrix[index] = tmp_a_constrain(r, c);
      // a_constraint_matrix[index] = a_constraint(r, c);
      // a_constraint_matrix[index] = tmp_a_constrain(r, c);
      index++;
    }
  }
  
  
  for (int i = 0; i < number_of_sampling_points_; ++i)
  {
    lower_bound[i] = lower_bound_vec[i];
    upper_bound[i] = upper_bound_vec[i];
    g_matrix[i] = tmp_b(i);
  }
  
  
  std::chrono::high_resolution_clock::time_point begin3_a = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < number_of_sampling_points_; ++i)
  {
    double p_x = reference_path.x_[i];
    double p_y = reference_path.y_[i];
    double yaw = reference_path.yaw_[i];
    double yaw_to_right_edge = yaw - M_PI/2;
    double yaw_to_left_edge = yaw + M_PI/2;
    double resolution = grid_map.getResolution();
    Eigen::Vector2d origin_p;
    origin_p << p_x, p_y;
    double cost;
    bool is_valid = grid_map.isInside(origin_p);
    if(is_valid)
    {
      cost = grid_map.atPosition(layer_name, origin_p);
    }
    else
    {
      std::cerr << "something wrong; reference point is outside map"  << std::endl;
    }
    
    
    
    double left_edge, right_edge;
    if(cost > 0.99999)
    {
      std::cerr << "centerpoint is occupied" << std::endl;
    }
    else
    {  
      double past_p_x = p_x;
      double past_p_y = p_y;
      for(int j = 0; j < 400; j++)
      {
        double new_p_x = past_p_x + std::cos(yaw_to_left_edge)*resolution;
        double new_p_y = past_p_y + std::sin(yaw_to_left_edge)*resolution;
        Eigen::Vector2d new_p;
        new_p << new_p_x, new_p_y;
        bool is_valid = grid_map.isInside(new_p);
        if(is_valid)
        {
          double cost = grid_map.atPosition(layer_name, new_p);
          // std::cerr << "cost " << cost << std::endl;
          if(cost>0.9999)
          {
            double distance = calculate2DDistace(origin_p, new_p);
            left_edge = distance;
            break;
          }
        }
        else
        {
          std::cerr << "reach left bound in map" << std::endl;
          Eigen::Vector2d past_p;
          past_p << past_p_x, past_p_y;
          double distance = calculate2DDistace(origin_p, past_p);
          left_edge = distance;
          break;
        }
        
        past_p_x = new_p_x;
        past_p_y = new_p_y;
      }
      
      for(int j = 0; j < 400; j++)
      {
        double new_p_x = past_p_x + std::cos(yaw_to_right_edge)*resolution;
        double new_p_y = past_p_y + std::sin(yaw_to_right_edge)*resolution;
        Eigen::Vector2d new_p;
        new_p << new_p_x, new_p_y;
        bool is_valid = grid_map.isInside(new_p);
        if(is_valid)
        {
          double cost = grid_map.atPosition(layer_name, new_p);
          if(cost>0.9999)
          {
            double distance = calculate2DDistace(origin_p, new_p);
            right_edge = distance;
            break;
          }
        }
        else
        {
          std::cerr << "reach right bound in map" << std::endl;
          Eigen::Vector2d past_p;
          past_p << past_p_x, past_p_y;
          double distance = calculate2DDistace(origin_p, past_p);
          right_edge = distance;
          break;
        }
        
        past_p_x = new_p_x;
        past_p_y = new_p_y;
      }
    }
    
    // std::cerr << "origin cost " << cost<< std::endl;
    // std::cerr << "left edge " << left_edge<< std::endl;
    // std::cerr << "right edge " << right_edge<< std::endl;
  }
  // 経過時間を取得
  std::chrono::high_resolution_clock::time_point end3_a = std::chrono::high_resolution_clock::now();  
  std::chrono::nanoseconds elapsed_time3_a = 
  std::chrono::duration_cast<std::chrono::nanoseconds>(end3_a - begin3_a);
  std::cout <<"make constraint " <<elapsed_time3_a.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  
  

  
  lower_bound[38] = 1;
  lower_bound[39] = 1;
  lower_bound[40] = 1;
  lower_bound[41] = 1;
  lower_bound[42] = 1;
  lower_bound[43] = 1;
  upper_bound[38] = 3;
  upper_bound[39] = 3;
  upper_bound[40] = 3;
  upper_bound[41] = 3;
  upper_bound[42] = 3;
  upper_bound[43] = 3;
  
  int max_iter = 500;
  
  std::chrono::high_resolution_clock::time_point begin3 = std::chrono::high_resolution_clock::now();
  if(!is_solver_initialized_)
  {
    auto ret = solver_ptr_->init(h_matrix, g_matrix, 
                          lower_bound, upper_bound, 
                          max_iter); 
    is_solver_initialized_ = true;
  }
  else
  {
    std::cerr << "warm start"  << std::endl;
    auto ret = solver_ptr_->hotstart(g_matrix, 
                                     lower_bound, upper_bound, max_iter);
  }
  
  
  double result[number_of_sampling_points_];
  solver_ptr_->getPrimalSolution(result);
  
  // 経過時間を取得
  std::chrono::high_resolution_clock::time_point end3 = std::chrono::high_resolution_clock::now();  
  std::chrono::nanoseconds elapsed_time3 = 
  std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - begin3);
  std::cout <<"solve " <<elapsed_time3.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  
  for(size_t i = 1; i < number_of_sampling_points_; i++)
  {
    geometry_msgs::Pose pose_in_lidar_tf;
    pose_in_lidar_tf.position.x = longitudinal_reference_point_vec[i];
    pose_in_lidar_tf.position.y = result[i];
    pose_in_lidar_tf.position.z = in_reference_waypoints_in_lidar.front().pose.pose.position.z;
    pose_in_lidar_tf.orientation.w = 1.0;
    geometry_msgs::Pose pose_in_map_tf;
    tf2::doTransform(pose_in_lidar_tf, pose_in_map_tf, lidar2map_tf);
    autoware_msgs::Waypoint waypoint;
    waypoint.pose.pose = pose_in_map_tf;
    out_waypoints.push_back(waypoint); 
  }
  

}