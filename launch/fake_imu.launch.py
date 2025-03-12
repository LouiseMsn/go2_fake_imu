from launch import LaunchDescription
from launch.actions import  DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():

    
    gravity_arg = DeclareLaunchArgument(
                'gravity',
                default_value=TextSubstitution(text='1'),
                description='1 or 0 : enables or not the gravity.'
    )
    debug_arg = DeclareLaunchArgument(
                'debug',
                default_value=TextSubstitution(text='0'),
                description='1 or 0 : enables debug output.'
    )
    publishing_freq_arg = DeclareLaunchArgument(
                'pub_freq',
                default_value=TextSubstitution(text='500'),
                description='Positive integer: frequency of the /lowstate topic publishing in Hertz.'
    )
    run_time_arg = DeclareLaunchArgument(
                'run_time',
                default_value=TextSubstitution(text='10'),
                description='Positive integer : Run time in seconds.'
    )


    return LaunchDescription([

        gravity_arg,
        debug_arg,
        publishing_freq_arg,
        run_time_arg,

        Node(
            package="go2_fake_imu",
            executable="fake_imu_node",
            name='fake_imu_node',
            parameters=[{
            "gravity": LaunchConfiguration('gravity'),
            "debug": LaunchConfiguration('debug'),
            "pub_freq": LaunchConfiguration('pub_freq'),
            "run_time": LaunchConfiguration('run_time')
        }]
        )
])