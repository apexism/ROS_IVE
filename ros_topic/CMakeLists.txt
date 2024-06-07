cmake_minimum_required(VERSION 3.0.2)
project(ros_topic)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

add_message_files(
  FILES
  Message1.msg
  #Message2.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ros_topic
 CATKIN_DEPENDS roscpp std_msgs
 DEPENDS system_lib
)

## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


##토픽 퍼블리셔 실행파일 이름,소스코드 지정
add_executable(ros_topic_publisher src/ros_topic_publisher.cpp)
add_dependencies(ros_topic_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_topic_publisher 
${catkin_LIBRARIES}
)
##토픽 서브스크리버 실행파일 이름,소스코드 지정
add_executable(ros_topic_subscriber src/ros_topic_subscriber.cpp)
add_dependencies(ros_topic_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ros_topic_subscriber 
${catkin_LIBRARIES}
)