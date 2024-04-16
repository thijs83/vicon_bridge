
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_definitions(-DROS_VERSION2)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Marker.msg"
  "msg/Markers.msg"
  DEPENDENCIES geometry_msgs std_msgs  # Add packages that above messages depend on
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(
    vicon_sdk/DataStream  # For Vicon SDK
    vicon_sdk             # For StreamCommon, required by ViconSDK
    include
    ${Boost_INCLUDE_DIRS}
)

# Compile Vicon SDK from scratch to avoid Boost version mismatch clashes
file(GLOB_RECURSE vicon_sdk_files "${CMAKE_CURRENT_SOURCE_DIR}/vicon_sdk/**/**.cpp")
add_library(vicon_sdk_lib ${vicon_sdk_files})
target_link_libraries(vicon_sdk_lib PUBLIC ${Boost_LIBRARIES})
install(TARGETS vicon_sdk_lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        PUBLIC_HEADER DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(vicon_sdk_lib)

file(GLOB_RECURSE LIBRARY_HEADERS "include/*.h")
add_executable(vicon_bridge_node src/vicon_bridge.cpp)
ament_target_dependencies(vicon_bridge_node
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  )
target_link_libraries(vicon_bridge_node vicon_sdk_lib ${Boost_LIBRARIES})
rosidl_target_interfaces(vicon_bridge_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


install(TARGETS
  vicon_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()