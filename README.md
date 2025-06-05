Clone the repository to the webots tutorial workspace. 

# Running the Project on Webots Simulator
In the workspace:


Terminal window 1 (world):
```
colcon build
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 launch my_package robot_launch.py
```

Terminal window 2 (drone):
```
colcon build
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run my_package mavic_node
```

Terminal window 3 (turtlebot):
```
colcon build
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
ros2 run my_package turtlebot_controller
```

# Running the Project on Real World
The real world version of this application is available at: https://github.com/hollip2023/Tello_TurtleBot_multiBot_move 

# Video demo
https://youtu.be/0KDNKakU43E
