cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kalman_filter
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

set(GLOG_LIB glog)

#target_link_libraries(${PROJECT_NAME} PUBLIC ${GLOG_LIB})

include_directories(
  include
  common
  transform
  sensor
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/kalman_filter.cpp
# )


find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
# find_package(cartographer REQUIRED)

if(PROTOBUF_FOUND)
  message(STATUS "protobuf library found")
else()
  message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

include_directories(${PROTOBUF_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
  proto/AddressBook.proto 
  proto/pose_tracker_options.proto
  proto/kalman_local_trajectory_builder_options.proto
  proto/real_time_correlative_scan_matcher_options.proto
  )

add_executable(kalman_filter_node src/kalman_filter_node.cpp ${PROTO_SRCS} ${PROTO_HDRS} 

 # src/kalman_local_trajectory_builder_options.cc
  src/kalman_local_trajectory_builder.cc
  src/pose_tracker.cc
  sensor/imu_tracker.cc
  sensor/odometry_tracker.cc
  common/customized_time.cc 
  
)
target_link_libraries(kalman_filter_node ${catkin_LIBRARIES} ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN_LIBRARIES} ${GLOG_LIB})

SET(SRC_LIST src/tst_protobuf.cpp)
ADD_EXECUTABLE(cppTest ${SRC_LIST} ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(cppTest ${PROTOBUF_LIBRARIES})

