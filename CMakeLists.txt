cmake_minimum_required(VERSION 2.8.3)
project(landing_vision)
add_definitions(-std=c++11 -g -O3)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(OpenCV_DIR ~/packages/opencv/build)
#set(Sophus_DIR ~/packages/sophus/build)

find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  genmsg 
  OpenCV
#  Sophus 
  cv_bridge 
  image_transport
  geometry_msgs
  message_generation
  visualization_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES  markerData.msg marker4c.msg markers.msg) #Num.msg
add_service_files(FILES imgGrab.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS geometry_msgs roscpp std_msgs message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(imgGrab src/imgGrab.cpp)
target_link_libraries(imgGrab ${catkin_LIBRARIES} ${OpenCV_LIBS} ) #${Sophus_LIBRARIES}
add_dependencies(imgGrab landing_vision_generate_messages_cpp)

add_executable(topicRcv src/topicRcv.cpp)
target_link_libraries(topicRcv ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(topicRcv landing_vision_generate_messages_cpp)

#add_executable(rviz_1_shape.cpp src/rviz_1_shape.cpp)
#target_link_libraries(rviz_1_shape ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(rviz_1_shape landing_vision_generate_messages_cpp)

add_executable(rviz_2_points src/rviz_2_points.cpp)
target_link_libraries(rviz_2_points ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(rviz_2_points landing_vision_generate_messages_cpp)

add_executable(drawRviz src/drawRviz.cpp)
target_link_libraries(drawRviz ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(drawRviz landing_vision_generate_messages_cpp)


#add_executable(poseEstimation src/poseEstimation.cpp)
#target_link_libraries(poseEstimation ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(poseEstimation landing_vision_generate_messages_cpp)

#add_executable(droneLanding src/droneLanding.cpp)
#target_link_libraries(droneLanding ${catkin_LIBRARIES} ${OpenCV_LIBS})
#add_dependencies(droneLanding landing_vision_generate_messages_cpp)

