# Multi Robot Navigation
## Multiple Turtlebot3 robot support in Gazebo
The ROS2 project  scalable solution for launching multiple TurtleBot3 robots with navigation capabilities using the Navigation2 (Nav2) stack. By leveraging namespaces in ROS2, this project enables the seamless deployment of multiple TurtleBot3 robots in a simple and organized manner. Each robot instance can be differentiated by its unique namespace, ensuring independence and preventing naming conflicts.

The 'master' branch includes an implementation that functions with the humble framework

'master' -> ROS2 Humble
```
mkdir -p robot_ws/src
cd robot_ws/src

# For Humble use master branch
git clone https://github.com/TonyOx369/Concordia-Research-Multibot/src/turtlebot3_multi_robot.git -b master

cd robot_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -r -y
```
## Run without nav2 stack
```
ros2 launch turtlebot3_multi_robot gazebo_multi_world.launch.py 
```
# turtlebot3_multi_robot

![image](https://github.com/arshadlab/turtlebot3_multi_robot/assets/85929438/fc958709-018d-48d2-b5b6-6674b53913c8)

## Run with nav2 stack

#### Robot Configuration

The arrangement of robots is configured in gazebo_multi_nav2_world.launch.py launch file. A potential future enhancement could involve retrieving the configurations from a file, such as json.

Names and poses for the robots in nav2 example
```
 robots = [
 {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
 {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01},
 {'name': 'tb3', 'x_pose': '1.5', 'y_pose': '-0.5', 'z_pose': 0.01},
 {'name': 'tb4', 'x_pose': '1.5', 'y_pose': '0.5', 'z_pose': 0.01},
 # …
 # …
 ]
```
```
ros2 launch turtlebot3_multi_robot gazebo_multi_nav2_world.launch.py 
```
```
ros2 run turtlebot3_multi_robot goal_listener_and_sender.py 
```
```
ros2 run turtlebot3_multi_robot spawn.py 
```


[three.webm](https://github.com/user-attachments/assets/97ac50bc-6281-464f-9f48-2ae6f4c0eb3a)


**References**: https://medium.com/@arshad.mehmood/a-guide-to-multi-robot-navigation-utilizing-turtlebot3-and-nav2-cd24f96d19c6


