from setuptools import find_packages, setup

package_name = 'turtlebot_controller'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/both_robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/turtle_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/mavic_webots.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/turtlebot_webots.urdf']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sandra',
    maintainer_email='svekho@utu.fi',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_controller = turtlebot_controller.simple_controller:main',
        ],
    },
)
