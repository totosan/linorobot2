from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='linorobot2_vision',
			executable='linorobot2_vision',
			name='linorobot2_vision',
			output='screen',
			parameters=[],
			remappings=[
				#('/image_raw', '/camera/color/image_raw'),  # Adjust the topic name if necessary
			]
		),
	])