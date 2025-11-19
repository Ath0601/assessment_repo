from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, RegisterEventHandler, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = FindPackageShare('pickplace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_path = PathJoinSubstitution([pkg_share, 'worlds', 'empty.world'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'xarm5', 'xarm5_robot.urdf.xacro'])

    robot_description_content = Command([
        FindExecutable(name='xacro'),
        ' ',
        xacro_file,
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(robot_description_content, value_type=str)
        }],
        output='screen'
    )
        # -------- RGB camera bridge --------
    bridge_rgb = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/xarm5/link/link5/sensor/rgb_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '--ros-args', '-r',
            '/world/empty/model/xarm5/link/link5/sensor/rgb_camera/image:=/camera/color/image_raw'
        ],
        output='screen'
    )

    delayed_bridge_rgb = TimerAction(
        period=7.0,
        actions=[bridge_rgb]
    )

    bridge_rgb_caminfo = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/world/empty/model/xarm5/link/link5/sensor/rgb_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '--ros-args', '-r',
        '/world/empty/model/xarm5/link/link5/sensor/rgb_camera/camera_info:=/camera/color/camera_info'
    ],
    output='screen'
    )

    delayed_bridge_caminfo = TimerAction(
        period=7.0,
        actions=[bridge_rgb_caminfo]
    )

    # -------- Depth camera bridge --------
    bridge_depth = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/empty/model/xarm5/link/link5/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '--ros-args', '-r',
            '/world/empty/model/xarm5/link/link5/sensor/rgbd_camera/depth_image:=/camera/depth/image_raw'
        ],
        output='screen'
    )

    delayed_bridge_depth = TimerAction(
        period=9.0,
        actions=[bridge_depth]
    )

    bridge_depth_caminfo = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/world/empty/model/xarm5/link/link5/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '--ros-args', '-r',
        '/world/empty/model/xarm5/link/link5/sensor/rgbd_camera/camera_info:=/camera/depth/camera_info'
    ],
    output='screen'
    )

    delayed_bridge_depth_caminfo = TimerAction(
        period=9.0,
        actions=[bridge_depth_caminfo]
    )

    bridge_box1 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/box1/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "--ros-args", "-r", "/model/box1/pose:=/box1/pose"
        ]
    )

    delayed_bridge_box1 = TimerAction(
        period=11.0,
        actions=[bridge_box1]
    )

    bridge_box2 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/box2/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "--ros-args", "-r", "/model/box2/pose:=/box2/pose"
        ]
    )
    delayed_bridge_box2 = TimerAction(
        period=11.0,
        actions=[bridge_box2]
    )

    bridge_box3 = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/box3/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
            "--ros-args", "-r", "/model/box3/pose:=/box3/pose"
        ]
    )
    delayed_bridge_box3 = TimerAction(
        period=11.0,
        actions=[bridge_box3]
    )

    bridge_pointcloud = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/model/xarm5/link/link5/sensor/rgbd_camera/points"
            "@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
            "--ros-args", "-r",
            "/world/empty/model/xarm5/link/link5/sensor/rgbd_camera/points:=/camera/depth/points"
        ],
        output="screen"
    )

    delayed_bridge_pointcloud = TimerAction(
        period=10.0,
        actions=[bridge_pointcloud]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])
        ]),
        launch_arguments={
            'gz_args': ['-r ', world_path],
            'on_exit_shutdown': 'true'
        }.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'xarm5', '-topic', 'robot_description'],
        output='screen'
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['xarm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('moveit_config'), 'launch', 'move_group.launch.py'])
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        gazebo,
        robot_state_publisher_node,
        spawn_robot,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    TimerAction(
                        period=4.0,
                        actions=[joint_state_spawner]
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[
                    TimerAction(
                        period=4.0,
                        actions=[trajectory_controller_spawner]
                    )
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=trajectory_controller_spawner,
                on_exit=[
                    TimerAction(
                        period=4.0,
                        actions=[gripper_controller_spawner]
                    )
                ]
            )
        ),
        moveit_launch,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[
                    delayed_bridge_rgb,
                    delayed_bridge_caminfo,
                    delayed_bridge_depth,
                    delayed_bridge_depth_caminfo,
                    delayed_bridge_box1,
                    delayed_bridge_box2,
                    delayed_bridge_box3,
                    delayed_bridge_pointcloud
                ]
            )
        )
    ])