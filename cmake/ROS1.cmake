
find_package(Boost REQUIRED COMPONENTS system thread)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
    roscpp
    tf2
    tf2_ros
    diagnostic_updater
)

add_definitions(-DROS_VERSION1)


# Generate messages and services
add_message_files(FILES
    Marker.msg
    Markers.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS 
    geometry_msgs
    message_runtime 
    roscpp 
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    vicon_sdk/DataStream  # For Vicon SDK
    vicon_sdk             # For StreamCommon, required by ViconSDK
    include
    ${catkin_INCLUDE_DIRS}
)


# Compile Vicon SDK from scratch to avoid Boost version mismatch clashes
file(GLOB_RECURSE vicon_sdk_files "${CMAKE_CURRENT_SOURCE_DIR}/vicon_sdk/**/**.cpp")
add_library(vicon_sdk ${vicon_sdk_files})
target_link_libraries(vicon_sdk PUBLIC ${Boost_LIBRARIES})

add_executable(vicon_bridge_node
    src/vicon_bridge.cpp
)
target_link_libraries(vicon_bridge_node
    vicon_sdk
    ${catkin_LIBRARIES}
)
add_dependencies(vicon_bridge_node ${PROJECT_NAME}_gencpp)

add_executable(testclient src/ViconDataStreamSDK_CPPTest.cpp)
target_link_libraries(testclient vicon_sdk)

# Install
install(TARGETS vicon_sdk vicon_bridge_node testclient
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


