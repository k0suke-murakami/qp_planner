/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apacpoints_marker_arrayion 2.0 (the "License");
 * you may not use this file except in compliance with thxpxoe License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <iostream>
#include <algorithm>
#include <vector>
#include <chrono>


#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


//TODO: in order to visualize text velocity
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>


#include <autoware_msgs/Lane.h>
#include <autoware_msgs/DetectedObjectArray.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include "qp_planner.h"
#include "vectormap_ros.h"
#include "vectormap_struct.h"
#include "calculate_center_line.h"
#include "modified_reference_path_generator.h"

#include "qp_planner_ros.h"

double calculate2DDistace(const geometry_msgs::Point& point1,
                          const geometry_msgs::Point& point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double distance = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
  return distance;
}

QPPlannerROS::QPPlannerROS()
  : nh_(), 
  private_nh_("~"),
  use_global_waypoints_as_center_line_(true),
  has_calculated_center_line_from_global_waypoints_(false),
  got_modified_reference_path_(false)
{
  double min_radius; 
  private_nh_.param<bool>("only_testing_modified_global_path", only_testing_modified_global_path_, false);
  private_nh_.param<double>("min_radius", min_radius, 1.6);
  qp_planner_ptr_.reset(new QPPlanner());
  modified_reference_path_generator_ptr_.reset(
    new ModifiedReferencePathGenerator(
      min_radius));
  
  tf2_buffer_ptr_.reset(new tf2_ros::Buffer());
  tf2_listner_ptr_.reset(new tf2_ros::TransformListener(*tf2_buffer_ptr_));
  
  
  safety_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("safety_waypoints", 1, true);
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("qp_planner_debug_markes", 1, true);
  gridmap_nav_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("gridmap_pointcloud", 1, true);
  final_waypoints_sub_ = nh_.subscribe("base_waypoints", 1, &QPPlannerROS::waypointsCallback, this);
  current_pose_sub_ = nh_.subscribe("/current_pose", 1, &QPPlannerROS::currentPoseCallback, this);
  current_velocity_sub_ = nh_.subscribe("/current_velocity", 1, &QPPlannerROS::currentVelocityCallback, this);
  objects_sub_ = nh_.subscribe("/detection/lidar_detector/objects", 1, &QPPlannerROS::objectsCallback, this);
  grid_map_sub_ = nh_.subscribe("/semantics/costmap", 1, &QPPlannerROS::gridmapCallback, this);
  // double timer_callback_dt = 0.05;
  double timer_callback_delta_second = 0.1;
  // double timer_callback_dt = 1.0;
  // double timer_callback_dt = 0.5;
  timer_ = nh_.createTimer(ros::Duration(timer_callback_delta_second), &QPPlannerROS::timerCallback, this);
}

QPPlannerROS::~QPPlannerROS()
{
}


void QPPlannerROS::waypointsCallback(const autoware_msgs::Lane& msg)
{
  if(in_pose_ptr_)
  {
    if(!in_waypoints_ptr_)
    {
      size_t nearest_wp_index;
      double min_dist = 9999999;
      //find nearest 
      for(size_t i = 0; i < msg.waypoints.size(); i++)
      {
        double px = msg.waypoints[i].pose.pose.position.x;
        double py = msg.waypoints[i].pose.pose.position.y;
        double ex = in_pose_ptr_->pose.position.x;
        double ey = in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(px-ex,2)+std::pow(py-ey, 2));
        if(distance < min_dist)
        {
          min_dist = distance;
          nearest_wp_index = i;
        }
      }
      
      autoware_msgs::Lane tmp_lane;
      tmp_lane.header = msg.header;
      for (size_t i = nearest_wp_index; i < msg.waypoints.size(); i++)
      {
        tmp_lane.waypoints.push_back(msg.waypoints[i]);
      }
      in_waypoints_ptr_.reset(new autoware_msgs::Lane(tmp_lane));
      previous_nearest_wp_index_.reset(new size_t(nearest_wp_index));
    }
    else
    {
      size_t nearest_wp_index;
      double min_dist = 9999999;
      for(size_t i = *previous_nearest_wp_index_; 
                 i < *previous_nearest_wp_index_ +20;
                 i++)
      {
        double px = msg.waypoints[i].pose.pose.position.x;
        double py = msg.waypoints[i].pose.pose.position.y;
        double ex = in_pose_ptr_->pose.position.x;
        double ey = in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(px-ex,2)+std::pow(py-ey, 2));
        if(distance < min_dist)
        {
          min_dist = distance;
          nearest_wp_index = i;
        }
        autoware_msgs::Lane tmp_lane;
        tmp_lane.header = msg.header;
        for (size_t i = nearest_wp_index; i < msg.waypoints.size(); i++)
        {
          tmp_lane.waypoints.push_back(msg.waypoints[i]);
        }
        in_waypoints_ptr_.reset(new autoware_msgs::Lane(tmp_lane));
        previous_nearest_wp_index_.reset(new size_t(nearest_wp_index));
      }
    }
  }
}

void QPPlannerROS::currentPoseCallback(const geometry_msgs::PoseStamped & msg)
{
  in_pose_ptr_.reset(new geometry_msgs::PoseStamped(msg));
}

void QPPlannerROS::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  in_twist_ptr_.reset(new geometry_msgs::TwistStamped(msg));
}

void QPPlannerROS::gridmapCallback(const grid_map_msgs::GridMap& msg)
{ 
  if(in_waypoints_ptr_)
  {
    geometry_msgs::TransformStamped gridmap2map_tf;
    geometry_msgs::TransformStamped map2gridmap_tf;
    try
    {
        gridmap2map_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  in_waypoints_ptr_->header.frame_id, 
          /*src*/ msg.info.header.frame_id,
          ros::Time(0));
        gridmap2map_tf_.reset(new geometry_msgs::TransformStamped(gridmap2map_tf));
        map2gridmap_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  msg.info.header.frame_id, 
          /*src*/ in_waypoints_ptr_->header.frame_id,
          ros::Time(0));
        map2gridmap_tf_.reset(new geometry_msgs::TransformStamped(map2gridmap_tf));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    in_gridmap_ptr_.reset(new grid_map_msgs::GridMap(msg));
  }
}

void QPPlannerROS::objectsCallback(const autoware_msgs::DetectedObjectArray& msg)
{
  if(in_waypoints_ptr_)
  {
    if(msg.objects.size() == 0)
    {
      return;
    }
    geometry_msgs::TransformStamped lidar2map_tf;
    try
    {
        lidar2map_tf = tf2_buffer_ptr_->lookupTransform(
          /*target*/  in_waypoints_ptr_->header.frame_id, 
          /*src*/ msg.header.frame_id,
          ros::Time(0));
        lidar2map_tf_.reset(new geometry_msgs::TransformStamped(lidar2map_tf));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
    in_objects_ptr_.reset(new autoware_msgs::DetectedObjectArray(msg));
    in_objects_ptr_->header.frame_id = in_waypoints_ptr_->header.frame_id;
    for(auto& object: in_objects_ptr_->objects)
    {
      object.header.frame_id = in_waypoints_ptr_->header.frame_id;
      geometry_msgs::PoseStamped current_object_pose;
      current_object_pose.header = object.header;
      current_object_pose.pose = object.pose;
      geometry_msgs::PoseStamped transformed_pose;
      tf2::doTransform(current_object_pose, transformed_pose, *lidar2map_tf_);
      object.pose = transformed_pose.pose;
    }
  }
}

void QPPlannerROS::timerCallback(const ros::TimerEvent &e)
{
  if(!in_pose_ptr_)
  {
    std::cerr << "pose not arrive" << std::endl;
  }
  if(!in_twist_ptr_)
  {
    std::cerr << "twist not arrive" << std::endl;
  }
  if(!in_waypoints_ptr_)
  {
    std::cerr << "waypoints not arrive" << std::endl;
  }
  if(!in_gridmap_ptr_)
  {
    std::cerr << "costmap not arrive" << std::endl;
  }
  
  if(in_pose_ptr_ && 
     in_twist_ptr_ && 
     in_waypoints_ptr_ && 
     in_gridmap_ptr_) 
  { 
    // 1. 現在日時を取得
    std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
    
    grid_map::GridMap grid_map;
    grid_map::GridMapRosConverter::fromMessage(*in_gridmap_ptr_, grid_map);
    
    // trigger when call generateModifiedReferencePath
    //calculate s between nearest reference path point and first reference path point
    geometry_msgs::Point incremental_start_point;
    geometry_msgs::Point incremental_goal_point;
    if(incremental_reference_path_in_map_ptr_)
    {
      geometry_msgs::Point first_reference_point =
       incremental_reference_path_in_map_ptr_->front().pose.pose.position;
      double min_dist = 99999;
      size_t nearest_wp_index = 0;
      for(size_t i=0; i< incremental_reference_path_in_map_ptr_->size(); i++)
      {
        double px = incremental_reference_path_in_map_ptr_->at(i).pose.pose.position.x;
        double py = incremental_reference_path_in_map_ptr_->at(i).pose.pose.position.y;
        double ex = in_pose_ptr_->pose.position.x;
        double ey = in_pose_ptr_->pose.position.y;
        double distance = std::sqrt(std::pow(px-ex, 2)+ std::pow(py-ey, 2));
        if(distance < min_dist)
        {
          min_dist = distance;
          nearest_wp_index = i;
        }
      }
      
      std::vector<autoware_msgs::Waypoint> tmp_waypoints_in_map;
      std::vector<autoware_msgs::Waypoint> tmp_waypoints_in_gridmap;
      for(size_t i = nearest_wp_index; i < incremental_reference_path_in_map_ptr_->size(); i++)
      {
        tmp_waypoints_in_map.push_back(incremental_reference_path_in_map_ptr_->at(i));
        tmp_waypoints_in_gridmap.push_back(incremental_reference_path_in_gridmap_ptr_->at(i));
      }
      incremental_reference_path_in_map_ptr_.reset(
        new std::vector<autoware_msgs::Waypoint>(tmp_waypoints_in_map));
      incremental_reference_path_in_gridmap_ptr_.reset(
        new std::vector<autoware_msgs::Waypoint>(tmp_waypoints_in_gridmap));
      
      geometry_msgs::Point past_p = 
        incremental_reference_path_in_map_ptr_->front().pose.pose.position;
      double accumulated_distance = 0;
      for(size_t i = 0; i < incremental_reference_path_in_map_ptr_->size(); i++)
      {
        double distance = calculate2DDistace(
                           incremental_reference_path_in_map_ptr_->at(i).pose.pose.position,
                           past_p);
        past_p = incremental_reference_path_in_map_ptr_->at(i).pose.pose.position;
        accumulated_distance += distance;
      }
      std::cerr << "incremental path size " << incremental_reference_path_in_gridmap_ptr_->size() << std::endl;
      // std::cerr << "accumulated dist " << accumulated_distance << std::endl;
      if(accumulated_distance < 30)
      {
        // // 1. 現在日時を取得
        std::chrono::high_resolution_clock::time_point begin_incremental = 
        std::chrono::high_resolution_clock::now();
        
        incremental_start_point = incremental_reference_path_in_map_ptr_->back().pose.pose.position;
        double min_dist = 99999;
        size_t nearest_wp_index = 0;
        for(size_t i = 0; i < 30; i++)
        {
          double dist = calculate2DDistace(
             in_waypoints_ptr_->waypoints[i].pose.pose.position, 
             incremental_start_point);
          if(dist < min_dist)
          {
            min_dist =dist;
            nearest_wp_index = i;
          }
        }
        
        double min_dist_to_goal = 99999;
        for(size_t i= nearest_wp_index; i < nearest_wp_index + 30; i++)
        {
          double dist = calculate2DDistace(
             in_waypoints_ptr_->waypoints[i].pose.pose.position, 
             incremental_start_point);
          if(dist < min_dist_to_goal && dist > 3)
          {
            min_dist_to_goal =dist;
            incremental_goal_point = in_waypoints_ptr_->waypoints[i].pose.pose.position;
          }
        }
        std::cerr << "min dist " << min_dist << std::endl;
        std::cerr << "min dist " << min_dist_to_goal << std::endl;
        geometry_msgs::Point point1_in_gridmap;
        tf2::doTransform(incremental_start_point, point1_in_gridmap, *map2gridmap_tf_);
        geometry_msgs::Point point2_in_gridmap;
        tf2::doTransform(incremental_goal_point, point2_in_gridmap, *map2gridmap_tf_);
        
        grid_map::Position position;
        position << point2_in_gridmap.x , point2_in_gridmap.y;
        if(!grid_map.isInside(position))
        {
          std::cerr << "goal point is outside gridmap"  << std::endl;
          // return;
        }
        else
        {
          
          double center_x = (point1_in_gridmap.x+point2_in_gridmap.x)/2.0;
          double center_y = (point1_in_gridmap.y+point2_in_gridmap.y)/2.0;
          double length_x_buffer = 3.0;
          double length_x = (point2_in_gridmap.x - point1_in_gridmap.x)+length_x_buffer;
          double length_y = point2_in_gridmap.y - point1_in_gridmap.y;
          
          grid_map::Position center_p;
          grid_map::Length length;
          center_p << center_x, center_y;
          // double bigger_length = std::max(10., length_y);
          length << std::max(static_cast<double>(10), length_x),
          std::max(static_cast<double>(15), length_y);
          bool is_success;
          //TODO: get all layers submap; it is not efficient 
          grid_map::GridMap submap =  grid_map.getSubmap(center_p, length, is_success);
          
          //debug; visualization
          nav_msgs::OccupancyGrid out_occupancy_grid;
          grid_map::GridMapRosConverter::toOccupancyGrid(submap, 
                                                        submap.getLayers().back(),
                                                        0,
                                                        1,
                                                        out_occupancy_grid);
          out_occupancy_grid.header = in_gridmap_ptr_->info.header;
          gridmap_nav_pub_.publish(out_occupancy_grid);
          //debug end
          std::vector<autoware_msgs::Waypoint> modified_reference_path_in_map;
          std::vector<autoware_msgs::Waypoint> modified_reference_path_in_gridmap;
          bool got_modified_incremental_reference_path =  
            modified_reference_path_generator_ptr_->generateModifiedReferencePath(
                submap,
                incremental_start_point,
                incremental_goal_point,
                *gridmap2map_tf_,
                *map2gridmap_tf_,
                modified_reference_path_in_map,
                modified_reference_path_in_gridmap);
          std::cerr << "start " << incremental_start_point.x << " "<<
                                  incremental_start_point.y << std::endl;
          std::cerr << "goal " << incremental_goal_point.x << " "<<
                                  incremental_goal_point.y << std::endl;
          if(got_modified_incremental_reference_path)
          {
            for(const auto& point: modified_reference_path_in_map)
            {
              std::cerr << "point " << point.pose.pose.position.x << " "
                                    << point.pose.pose.position.y << std::endl;
            }
          }
          else
          {
            std::cerr << "fails to find a path"  << std::endl;
          }
          
          incremental_reference_path_in_map_ptr_->insert(
            incremental_reference_path_in_map_ptr_->end(),
            modified_reference_path_in_map.begin(),
            modified_reference_path_in_map.end());
          incremental_reference_path_in_gridmap_ptr_->insert(
            incremental_reference_path_in_gridmap_ptr_->end(),
            modified_reference_path_in_gridmap.begin(),
            modified_reference_path_in_gridmap.end());
            
          cached_incremental_reference_path_in_map_ptr_->insert(
            cached_incremental_reference_path_in_map_ptr_->end(),
            modified_reference_path_in_map.begin(),
            modified_reference_path_in_map.end());
          cached_incremental_reference_path_in_gridmap_ptr_->insert(
            cached_incremental_reference_path_in_gridmap_ptr_->end(),
            modified_reference_path_in_gridmap.begin(),
            modified_reference_path_in_gridmap.end());
          
          // 3. 現在日時を再度取得
          std::chrono::high_resolution_clock::time_point end_incremental = 
          std::chrono::high_resolution_clock::now();
          // 経過時間を取得
          std::chrono::nanoseconds incremental_time = 
          std::chrono::duration_cast<std::chrono::nanoseconds>(end_incremental - begin_incremental);
          std::cout <<"incremental time " <<incremental_time.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
        }
      }
    }
    else
    {
      
      // 1. 現在日時を取得
      std::chrono::high_resolution_clock::time_point begin_incremental = 
      std::chrono::high_resolution_clock::now();
      
      incremental_start_point =  in_waypoints_ptr_->waypoints.at(5).pose.pose.position;
      geometry_msgs::Point point1_in_gridmap;
      tf2::doTransform(incremental_start_point, point1_in_gridmap, *map2gridmap_tf_);
      incremental_goal_point =  in_waypoints_ptr_->waypoints.at(8).pose.pose.position;
      geometry_msgs::Point point2_in_gridmap;
      tf2::doTransform(incremental_goal_point, point2_in_gridmap, *map2gridmap_tf_);
      
      
      grid_map::Position position;
      position << point2_in_gridmap.x , point2_in_gridmap.y;
      if(!grid_map.isInside(position))
      {
        std::cerr << "goal point is outside gridmap"  << std::endl;
        return;
      }
      
      double center_x = (point1_in_gridmap.x+point2_in_gridmap.x)/2.0;
      double center_y = (point1_in_gridmap.y+point2_in_gridmap.y)/2.0;
      double length_x_buffer = 3.0;
      double length_x = (point2_in_gridmap.x - point1_in_gridmap.x)+length_x_buffer;
      double length_y = point2_in_gridmap.y - point1_in_gridmap.y;
      
      grid_map::Position center_p;
      grid_map::Length length;
      center_p << center_x, center_y;
      // double bigger_length = std::max(10., length_y);
      length << length_x, std::max(static_cast<double>(10), length_y);
      bool is_success;
      //TODO: get all layers submap; it is not efficient 
      grid_map::GridMap submap =  grid_map.getSubmap(center_p, length, is_success);
      
      
      //debug; visualization
      nav_msgs::OccupancyGrid out_occupancy_grid;
      grid_map::GridMapRosConverter::toOccupancyGrid(submap, 
                                                    submap.getLayers().back(),
                                                    0,
                                                    1,
                                                    out_occupancy_grid);
      out_occupancy_grid.header = in_gridmap_ptr_->info.header;
      gridmap_nav_pub_.publish(out_occupancy_grid);
      //debug end
      
      std::vector<autoware_msgs::Waypoint> modified_reference_path_in_map;
      std::vector<autoware_msgs::Waypoint> modified_reference_path_in_gridmap;
      bool got_modified_incremental_reference_path =  
        modified_reference_path_generator_ptr_->generateModifiedReferencePath(
            submap,
            incremental_start_point,
            incremental_goal_point,
            *gridmap2map_tf_,
            *map2gridmap_tf_,
            modified_reference_path_in_map,
            modified_reference_path_in_gridmap);
      std::cerr << "size of incremental path " << modified_reference_path_in_map.size() << std::endl;
      incremental_reference_path_in_map_ptr_.reset(
        new std::vector<autoware_msgs::Waypoint>(modified_reference_path_in_map));
      incremental_reference_path_in_gridmap_ptr_.reset(
        new std::vector<autoware_msgs::Waypoint>(modified_reference_path_in_gridmap));
      
      cached_incremental_reference_path_in_map_ptr_.reset(
        new std::vector<autoware_msgs::Waypoint>(modified_reference_path_in_map));
      cached_incremental_reference_path_in_gridmap_ptr_.reset(
        new std::vector<autoware_msgs::Waypoint>(modified_reference_path_in_gridmap));
        
      // 3. 現在日時を再度取得
      std::chrono::high_resolution_clock::time_point end_incremental = 
      std::chrono::high_resolution_clock::now();
      // 経過時間を取得
      std::chrono::nanoseconds incremental_time = 
      std::chrono::duration_cast<std::chrono::nanoseconds>(end_incremental - begin_incremental);
      std::cout <<"incremental time " <<incremental_time.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
    }
    
    // if(cached_incremental_reference_path_in_gridmap_ptr_)
    // {
    // std::cerr << "cache size " << cached_incremental_reference_path_in_gridmap_ptr_->size() << std::endl;
      
    // }
    
    
    //begin qp_planner
    //first convert to gridmap freme
    // input_waypoints_ = *incremental_reference_path_in_map_ptr_
    input_waypoints_.clear();
    for(const auto& point: *incremental_reference_path_in_map_ptr_)
    {
      geometry_msgs::Pose pose;
      tf2::doTransform(point.pose.pose, pose, *map2gridmap_tf_);
      autoware_msgs::Waypoint tmp_wp;
      tmp_wp.pose.pose = pose;
      input_waypoints_.push_back(tmp_wp);
    }
    //second compensate points
    geometry_msgs::Point past_reference_point = input_waypoints_.front().pose.pose.position;
    double accumulated_distance = 0;
    for(const auto& point: input_waypoints_)
    {
      geometry_msgs::Point reference_point = point.pose.pose.position;
      double distance = calculate2DDistace(reference_point, past_reference_point);
      accumulated_distance += distance;
      past_reference_point = reference_point;
    }
    std::cerr << "current acc dist " << accumulated_distance << std::endl;
    
    if(accumulated_distance < 30)
    {
      double extra_dist = 30 - accumulated_distance;
      std::cerr << "extra dist " << extra_dist << std::endl;
      geometry_msgs::Point extra_point;
      extra_point.x = input_waypoints_.front().pose.pose.position.x - extra_dist;
      extra_point.y = input_waypoints_.front().pose.pose.position.y;
      extra_point.z = input_waypoints_.front().pose.pose.position.z;
      
      autoware_msgs::Waypoint dummy_wp = incremental_reference_path_in_gridmap_ptr_->back();
      dummy_wp.pose.pose.position = extra_point;
      input_waypoints_.insert(input_waypoints_.begin(), dummy_wp);
    }
    
    // if(accumulated_distance < 30)
    // {
    //   geometry_msgs::Point debug_past_reference_point = input_waypoints_in_gridmap.front().pose.pose.position;
    //   double debug_accumulated_distance = 0;
    //   for(const auto& point: input_waypoints_in_gridmap)
    //   {
    //     geometry_msgs::Point reference_point = point.pose.pose.position;
    //     double distance = calculate2DDistace(reference_point, debug_past_reference_point);
    //     debug_accumulated_distance += distance;
    //     debug_past_reference_point = reference_point;
    //   }
    //   std::cerr << "debug acc dist " << debug_accumulated_distance << std::endl;
    // }
    
    std::vector<autoware_msgs::Waypoint> out_waypoints;
    // qp_planner_ptr_->doPlan(*gridmap2map_tf_,
    //                         *in_pose_ptr_,
    //                         grid_map,
    //                         *incremental_reference_path_in_gridmap_ptr_,
    //                         out_waypoints);
    std::cerr << "--------------" << std::endl;
    
    
    //debug; marker array
    visualization_msgs::MarkerArray points_marker_array;
    int unique_id = 0;
    
    // // visualize debug modified reference point
    // visualization_msgs::Marker debug_modified_reference_points;
    // debug_modified_reference_points.lifetime = ros::Duration(0.2);
    // debug_modified_reference_points.header = in_pose_ptr_->header;
    // debug_modified_reference_points.ns = std::string("debug_modified_reference_points");
    // debug_modified_reference_points.action = visualization_msgs::Marker::MODIFY;
    // debug_modified_reference_points.pose.orientation.w = 1.0;
    // debug_modified_reference_points.id = unique_id;
    // debug_modified_reference_points.type = visualization_msgs::Marker::SPHERE_LIST;
    // debug_modified_reference_points.scale.x = 0.9;
    // debug_modified_reference_points.color.r = 1.0f;
    // debug_modified_reference_points.color.g = 1.0f;
    // debug_modified_reference_points.color.a = 1;
    // for(const auto& waypoint: *modified_reference_path_ptr_)
    // {
    //   debug_modified_reference_points.points.push_back(waypoint.pose.pose.position);
    // }
    // points_marker_array.markers.push_back(debug_modified_reference_points);
    // unique_id++;
    
    // visualize gridmap point
    visualization_msgs::Marker debug_incremental_goal_point;
    debug_incremental_goal_point.lifetime = ros::Duration(0.2);
    debug_incremental_goal_point.header = in_pose_ptr_->header;
    debug_incremental_goal_point.ns = std::string("debug_incremental_goal_point");
    debug_incremental_goal_point.action = visualization_msgs::Marker::MODIFY;
    debug_incremental_goal_point.pose.orientation.w = 1.0;
    debug_incremental_goal_point.id = unique_id;
    debug_incremental_goal_point.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_incremental_goal_point.scale.x = 0.9;
    debug_incremental_goal_point.color.r = 1.0f;
    debug_incremental_goal_point.color.g = 0.0f;
    debug_incremental_goal_point.color.a = 1;
    
    debug_incremental_goal_point.points.push_back(incremental_start_point);
    debug_incremental_goal_point.points.push_back(incremental_goal_point);
    
    points_marker_array.markers.push_back(debug_incremental_goal_point);
    unique_id++;
    
    // visualize gridmap point
    visualization_msgs::Marker debug_incremental_path;
    debug_incremental_path.lifetime = ros::Duration(0.2);
    debug_incremental_path.header = in_pose_ptr_->header;
    debug_incremental_path.ns = std::string("debug_incremental_path");
    debug_incremental_path.action = visualization_msgs::Marker::MODIFY;
    debug_incremental_path.pose.orientation.w = 1.0;
    debug_incremental_path.id = unique_id;
    debug_incremental_path.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_incremental_path.scale.x = 1.0f;
    debug_incremental_path.color.r = 1.0f;
    debug_incremental_path.color.g = 1.0f;
    debug_incremental_path.color.a = 1;
    
    for(const auto& waypoint: *incremental_reference_path_in_map_ptr_)
    {
      debug_incremental_path.points.push_back(waypoint.pose.pose.position);
    }
    
    points_marker_array.markers.push_back(debug_incremental_path);
    unique_id++;
    
    
    
    // visualize compensted points at qp_planner
    visualization_msgs::Marker debug_compensatec_ref_points;
    debug_compensatec_ref_points.lifetime = ros::Duration(0.2);
    debug_compensatec_ref_points.header = in_gridmap_ptr_->info.header;
    debug_compensatec_ref_points.ns = std::string("debug_compensatec_ref_points");
    debug_compensatec_ref_points.action = visualization_msgs::Marker::MODIFY;
    debug_compensatec_ref_points.pose.orientation.w = 1.0;
    debug_compensatec_ref_points.id = unique_id;
    debug_compensatec_ref_points.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_compensatec_ref_points.scale.x = 1.0f;
    debug_compensatec_ref_points.color.r = 1.0f;
    debug_compensatec_ref_points.color.g = 1.0f;
    debug_compensatec_ref_points.color.a = 1;
    for(const auto& waypoint: input_waypoints_)
    {
      debug_compensatec_ref_points.points.push_back(waypoint.pose.pose.position);
    }
    points_marker_array.markers.push_back(debug_compensatec_ref_points);
    unique_id++;
    
    
    //  // visualize out_waypoints
    // visualization_msgs::Marker qp_waypoints_marker;
    // qp_waypoints_marker.lifetime = ros::Duration(0.2);
    // qp_waypoints_marker.header = in_pose_ptr_->header;
    // qp_waypoints_marker.ns = std::string("qp_waypoints_marker");
    // qp_waypoints_marker.action = visualization_msgs::Marker::MODIFY;
    // qp_waypoints_marker.pose.orientation.w = 1.0;
    // qp_waypoints_marker.id = unique_id;
    // qp_waypoints_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    // qp_waypoints_marker.scale.x = 0.9;
    // qp_waypoints_marker.color.r = 1.0f;
    // qp_waypoints_marker.color.a = 0.6;
    // for(const auto& waypoint: out_waypoints)
    // {
    //   qp_waypoints_marker.points.push_back(waypoint.pose.pose.position);
    // }
    // points_marker_array.markers.push_back(qp_waypoints_marker);
    // unique_id++;
    
    
    markers_pub_.publish(points_marker_array);
    safety_waypoints_pub_.publish(*in_waypoints_ptr_);
  }
}