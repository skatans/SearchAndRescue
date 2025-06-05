Clone the repository to the webots tutorial workspace. 

# Running
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

The real world version of this application is available at: https://github.com/hollip2023/Tello_TurtleBot_multiBot_move 
