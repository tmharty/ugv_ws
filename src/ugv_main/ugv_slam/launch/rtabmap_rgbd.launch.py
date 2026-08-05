from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    queue_size = LaunchConfiguration('queue_size')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_queue_size = DeclareLaunchArgument(
        'queue_size', default_value='20',
        description='Queue size'
    )
    
    declare_qos = DeclareLaunchArgument(
        'qos', default_value='2',
        description='QoS used for input sensor topics'
    )
        
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Launch in localization mode.'
    )
                            
    # Parameters for the SLAM node
    parameters = {
            "frame_id": 'base_footprint',
            'queue_size': queue_size,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            'subscribe_scan': True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            # 1 Hz keyframes (the rtabmap default). Higher rates bloat the pose
            # graph, which slows loop closure more and more as the map grows —
            # too heavy for the Jetson Orin Nano at this robot's driving speed.
            "Rtabmap/DetectionRate": "1.0",
     }

    remappings = [
        ("rgb/image", "oak/rgb/image_rect"),
        ("rgb/camera_info", "oak/rgb/camera_info"),
        ("depth/image", "oak/stereo/image_raw"),
    ]
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Whether to launch RViz2 (rtabmap_viz is used when false)'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            get_package_share_directory('ugv_slam'), 'rviz', 'view_slam_3d.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Launch the oak lite bringup launch file
    bringup_oak_lite_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_vision'), 'launch'),
             '/oak_d_lite.launch.py']
        )
    )
    
    # SLAM mode:
    rtabmap_slam_node_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d']
    )  # This will delete the previous database (~/.ros/rtabmap.db)
            
    # Localization mode:
    rtabmap_slam_node_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[
            parameters,
            {'Mem/IncrementalMemory': 'False',
             'Mem/InitWMWithAllNodes': 'True'}
        ],
        remappings=remappings
    )
    
    # Launch the rtabmap viz node
    rtabmap_viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters],
        remappings=remappings,
        condition=UnlessCondition(LaunchConfiguration('use_rviz'))
    )

    # Launch the robot pose publisher launch file
    robot_pose_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
         '/robot_pose_publisher_launch.py'])
    ) 
                     
    return LaunchDescription([
        declare_use_sim_time,
        declare_queue_size,
        declare_qos,
        declare_localization,
        declare_use_rviz,
        rviz_node,
        bringup_oak_lite_launch,
        robot_pose_publisher_launch,
        rtabmap_slam_node_slam,
        rtabmap_slam_node_localization,
        rtabmap_viz_node
    ])
