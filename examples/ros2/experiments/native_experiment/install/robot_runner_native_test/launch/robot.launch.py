import launch
import launch_ros.actions

# arguments=["--number_of_cycles", "1"],
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="demo_nodes_py",
            node_executable="talker",
            node_name="talker",
            output="screen"),

        launch_ros.actions.Node(
            package="demo_nodes_py",
            node_executable="listener",
            node_name="listener",
            output="screen",
            on_exit=launch.actions.Shutdown())
    ])