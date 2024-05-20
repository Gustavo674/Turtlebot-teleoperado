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
            'teleop_interface = turtlebot_teleop.teleop_interface:main',
            'teleop_node = turtlebot_teleop.teleop_node:main',
        ],
    },
)
