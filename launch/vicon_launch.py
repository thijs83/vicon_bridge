from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('datastream_hostport', default_value='192.168.0.232'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('msg_type', default_value='geometry_msgs/msg/PoseStamped'),
        DeclareLaunchArgument('frequency_divider', default_value='1'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('reset_z_axis', default_value='true'),
        DeclareLaunchArgument('only_use_object_specific', default_value='true'),
        DeclareLaunchArgument('object_names', default_value='[drone1, rectangle2x1_0, rectangle2x1_1]'),
        DeclareLaunchArgument('object_msg_types', default_value='[geometry_msgs/msg/PoseStamped, geometry_msgs/msg/TransformStamped, geometry_msgs/msg/TransformStamped]'),
        DeclareLaunchArgument('object_frame_ids', default_value='[map, map, map]'),
        DeclareLaunchArgument('object_publish_topics', default_value='[/mavros/vision_pose/pose, /obstacles/rectangle2x1_0, /obstacles/rectangle2x1_1]'),
        DeclareLaunchArgument('object_frequency_divider', default_value='[1, 100, 100]'),

        Node(
            package='vicon_bridge',
            executable='vicon_bridge_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'stream_mode': 'ServerPush'},
                {'datastream_hostport': LaunchConfiguration('datastream_hostport')},
                {'frame_id': LaunchConfiguration('frame_id')},
                {'msg_type': LaunchConfiguration('msg_type')},
                {'frequency_divider': LaunchConfiguration('frequency_divider')},
                {'publish_transform': LaunchConfiguration('publish_tf')},
                {'reset_z_axis': LaunchConfiguration('reset_z_axis')},
                {'only_use_object_specific': LaunchConfiguration('only_use_object_specific')},
                {'object_specific/object_names': LaunchConfiguration('object_names')},
                {'object_specific/object_msg_types': LaunchConfiguration('object_msg_types')},
                {'object_specific/object_frame_ids': LaunchConfiguration('object_frame_ids')},
                {'object_specific/object_publish_topics': LaunchConfiguration('object_publish_topics')},
                {'object_specific/object_frequency_divider': LaunchConfiguration('object_frequency_divider')}
            ]
        )
    ])