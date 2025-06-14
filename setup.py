from setuptools import find_packages, setup

package_name = 'my_package'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/robot_launch.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/turtlebot_webots.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/mavic_webots.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/ros2control.yml']))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name + '/data', ['data/haarcascade_fullbody.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_robot_driver = my_package.my_robot_driver:main',
            'mavic_driver = my_package.mavic_driver:main',
            'mavic_node = my_package.mavic_node:main',
            'turtlebot_controller = my_package.turtlebot_controller:main'
        ],
    },
)
