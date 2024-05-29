from setuptools import setup

package_name = 'turtlebot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seu_nome',
    maintainer_email='seu_email@example.com',
    description='Teleoperation package for Turtlebot',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_node = turtlebot_teleop.teleop_node:main',
            'teleop_interface = turtlebot_teleop.teleop_interface:main',
            'obstacle_detection = turtlebot_teleop.obstacle_detection:main',
            'image_publisher = turtlebot_teleop.image_publisher:main',
            'image_subscriber = turtlebot_teleop.image_subscriber:main',
            'teleop_interface_gui = turtlebot_teleop.teleop_interface_gui:main',
        ],
    },
)
