from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace= "experiment", package='onsmc_rt', executable='time_publisher'),
        launch_ros.actions.Node(
            namespace= "experiment", package='onsmc_rt', executable='trajectory_generator'),
        launch_ros.actions.Node(
            namespace= "experiment", package='onsmc_rt', executable='controller'),
        launch_ros.actions.Node(
            namespace= "experiment", package='onsmc_rt', executable='reader'),
        launch_ros.actions.Node(
            namespace= "experiment", package='onsmc_rt', executable='commander'),
        launch_ros.actions.Node(
            namespace= "experiment", package='onsmc_rt', executable='writer'),
    ])