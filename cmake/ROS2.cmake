

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_definitions(-DROS_VERSION2)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Marker.msg"
  "msg/Markers.msg"
  DEPENDENCIES geometry_msgs std_msgs  # Add packages that above messages depend on
)
