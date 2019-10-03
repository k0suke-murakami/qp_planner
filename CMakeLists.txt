cmake_minimum_required(VERSION 2.8.3)
project(qp_planner)

add_compile_options(-std=c++11)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages


add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/distance_transform/extern/DopeVector ${CMAKE_CURRENT_BINARY_DIR}/lib_dope_vector)
include(${CMAKE_CURRENT_SOURCE_DIR}/lib/distance_transform/extern/DopeVector/sources_properties.cmake)


set(toLink DopeVector)
if(CMAKE_COMPILER_IS_GNUCXX)
find_package(Threads REQUIRED)
list(APPEND toLink Threads::Threads)
endif()



set(hdr_dir ${CMAKE_CURRENT_SOURCE_DIR}/lib/distance_transform/include)

set(hdr_inline_files
${hdr_dir}/distance_transform/inlines/distance_transform.inl
)
set_source_files_properties(${hdr_inline_files} PROPERTIES XCODE_EXPLICIT_FILE_TYPE "sourcecode.cpp.h")
source_group("distance_transform\\inlines" FILES ${hdr_inline_files})

set(hdr_main_files
${hdr_dir}/distance_transform/distance_transform.hpp
)
source_group("distance_transform" FILES ${hdr_main_files})

set(all_hdr ${hdr_inline_files} ${hdr_main_files})



add_library(distance_transform INTERFACE)

target_include_directories(distance_transform INTERFACE ${hdr_dir})
target_link_libraries(distance_transform INTERFACE ${toLink})

if(ATTACH_SOURCES)
target_sources(distance_transform INTERFACE ${all_hdr})
endif()

find_package(autoware_build_flags REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf2_ros
  autoware_msgs
  vector_map
  grid_map_ros
  qpoases_vendor
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  std_msgs
  tf2_ros
  autoware_msgs
  vector_map
  grid_map_ros
  qpoases_vendor
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${qpoases_vendor_INCLUDE_DIRS}/qpoases_vendor
  # ${CMAKE_SOURCE_DIR}/lib/osqp-0.6.0-linux64/include
  # /home/kosuke/autoware.ai/src/autoware/qp_planner/lib/osqp-0.6.0-linux64/include
  # ${CMAKE_SOURCE_DIR}/lib/osqp-0.5.0-linux64/include
  ${CMAKE_SOURCE_DIR}/lib/osqp-0.6.0-linux64/include
  
  # /home/kosuke/autoware.ai/src/autoware/qp_planner/lib/osqp-0.6.0-linux64/iib
)


add_executable(qp_planner
src/qp_planner_node.cpp
src/qp_planner_ros.cpp
src/qp_planner.cpp
src/vectormap_ros.cpp
src/calculate_center_line.cpp
src/modified_reference_path_generator.cpp
)

target_link_libraries(qp_planner
# ${catkin_LIBRARIES}
# ${CMAKE_DL_LIBS}
distance_transform
#   ${CMAKE_SOURCE_DIR}/lib/osqp-0.6.0-linux64/lib/libosqp.a
  ${catkin_LIBRARIES}
  ${CMAKE_SOURCE_DIR}/lib/osqp-0.6.0-linux64/lib/libosqp.a
  ${CMAKE_DL_LIBS}
  # ${CMAKE_SOURCE_DIR}/lib/osqp-0.6.0-linux64/lib/libosqp.so
  # lib/osqp-0.6.0-linux64/lib/libosqp.so
  # /home/kosuke/autoware.ai/src/autoware/qp_planner/lib/osqp-0.6.0-linux64/lib/libosqp.so
  # /home/kosuke/autoware.ai/src/autoware/qp_planner/lib/osqp-0.6.0-linux64/lib/libosqp.a
)

# include_directories(
#         ${CMAKE_SOURCE_DIR}/include/smooth_path_generator
#         ${CMAKE_SOURCE_DIR}/lib/osqp-0.5.0-linux64/include
#         #PRIVATE ${PYTHON_INCLUDE_DIRS}
#         ${autoware_config_msgs_INCLUDE_DIRS}
#         ${autoware_msgs_INCLUDE_DIRS}
#         ${catkin_INCLUDE_DIRS}
# )
# )

# target_link_libraries(smooth_path_generator
#         ${catkin_LIBRARIES}
#         ${CMAKE_SOURCE_DIR}/lib/osqp-0.5.0-linux64/lib/libosqp.a
#         ${CMAKE_DL_LIBS}
#         #${PYTHON_LIBRARIES}
# )
  
  add_dependencies(qp_planner
  ${catkin_EXPORTED_TARGETS}
  distance_transform
  # /home/kosuke/autoware.ai/src/autoware/qp_planner/lib/osqp-0.6.0-linux64/lib/libosqp.so
  # ${CMAKE_SOURCE_DIR}/lib/osqp-0.5.0-linux64/lib/libosqp.a
  # ${CMAKE_SOURCE_DIR}/lib/osqp-0.5.0-linux64/lib/libosqp.so
  )
  
TARGET_INCLUDE_DIRECTORIES(qp_planner PUBLIC
            /home/kosuke/autoware.ai/src/autoware/qp_planner/lib/osqp-0.6.0-linux64/include
            # /home/kosuke/autoware.ai/src/autoware/qp_planner/lib/osqp-0.6.0-linux64/lib
            include
            )
            
  install(TARGETS
  qp_planner
  distance_transform
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  
install(DIRECTORY launch/ lib/osqp-0.6.0-linux64/include
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

install(FILES ${CMAKE_SOURCE_DIR}/lib/osqp-0.6.0-linux64/lib/libosqp.a
DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
      