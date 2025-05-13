# This launches the nodes needed to run Sami Trivia.
# These are the tools, not the main UI, that must be launched in a seperate window via ros2 run


import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        # sami control node (arduino / json)
        launch_ros.actions.Node(
            package='sami_trivia',
            executable='sami_control',
            name='sami_control',
        ),
        # Wyatt's question bank node
        launch_ros.actions.Node(
            package='sami_trivia',
            executable='get_question',
            name='get_question',
        ),
        # Voice, Listen, Check Answer
        launch_ros.actions.Node(
            package='sami_trivia',
            executable='game_voice',
            name='game_voice',
        ),
        ])
    

'''
        # question testing
        launch_ros.actions.Node(
            package='sami_trivia',
            executable='q_test',
            name='q_test',
        ),
'''