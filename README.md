# Simulation_gazebo
Simulation of obstacle avoidance method in gazebo using turtlebot3.

In this project ROS2 Foxy is used as targer ROS version. Target hw platform -> TurtleBot3 burger.


## Building and running project

`source /opt/ros/foxy/setup.bash`

`export TURTLEBOT3_MODEL=burger`

`colcon build`

It is necessary to provide path to compiled libs. Note: dont forget to provide correct path.

`export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/ihor/Simulation_gazebo/install/assignment_3a/lib/assignment_3a`

`source install/setup.sh`

`ros2 launch assignment_3a 
obstacle_avoidance.launch.py`
