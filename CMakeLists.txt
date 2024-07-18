cmake_minimum_required(VERSION 3.5)
project(bt_random_controller)

set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(ZMQ)
if(ZMQ_FOUND)
    message(STATUS "ZeroMQ found.")
    add_definitions(-DZMQ_FOUND)
else()
  message(WARNING "ZeroMQ NOT found. Not including PublisherZMQ.")
endif()

set(CMAKE_CXX_STANDARD 17)

set(dependencies
    rclcpp
    behaviortree_cpp_v3
    sensor_msgs
    geometry_msgs
    ament_index_cpp
)

include_directories(include ${ZMQ_INCLUDE_DIRS})

add_library(forward_bt_node SHARED src/bt_random_controller/Forward.cpp)
add_library(back_bt_node SHARED src/bt_random_controller/Back.cpp)
add_library(turn_bt_node SHARED src/bt_random_controller/Turn.cpp)
add_library(is_obstacle_bt_node SHARED src/bt_random_controller/IsObstacle.cpp)

list(APPEND plugin_libs
  forward_bt_node
  back_bt_node
  turn_bt_node
  is_obstacle_bt_node
)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(bt_random_controller src/bt_random_controller_main.cpp)
ament_target_dependencies(bt_random_controller ${dependencies})
target_link_libraries(bt_random_controller ${ZMQ_LIBRARIES})

install(TARGETS
  ${plugin_libs}
  bt_random_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY behavior_tree_xml 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})

ament_package()
