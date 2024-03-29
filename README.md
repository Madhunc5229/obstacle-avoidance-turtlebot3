[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)
# obstacle-avoidance-turtlebot3

## This is a simple obstacle avoidance program using turtlebot3 in ROS2 (Humble), simulated in Gazebo   

## Dependencies

- Ubuntu 20.04 LTS / 22.04 LTS
- ROS2 Humble
- colcon
- rosdep
- turtlebot3

## Install ROS2 Humble 
- If you are using ubuntu 22.02, you can install ROS2 Humble with single command. If you are using ubuntu 20.04, then you will have install it by building from source.
- Please follow the steps mentioned in the following link to install ROS2 Humble.

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

## Install turtlebot package 
- If you have already installed turtlebot3 for ros2, please skip this step
- The following is debian package install for turtlebot3
```
sudo apt install ros-humble-turtlebot3*
```
## Building the package

### source the ROS2 setup bash.
```
source /opt/ros/humble/setup.bash
```
### Clone the repository
```
cd <ros2 workspace folder>/src
git clone https://github.com/Madhunc5229/obstacle-avoidance-turtlebot3.git
```

### Build the package using colcon
```
cd ..
colcon build --packages-select obstacle-avoidance-turtlebot3
```

### Source the package after building
```
source install/setup.bash
```
## Launching the package
- Launch the turtlebot3 and the bot_node which subscribes to `"/scan"` topic and publishes to `"/cmd_vel"` topic
- Please note that there is parameter `record` which has to be set to either true or false to record rosbad files for 30 seconds
```
ros2 launch obstacle-avoidance-turtlebot3 obsAvoid.launch record:=true
```
## Rosbag 
- To inspect the rosbag file, run the following
```
ros2 bag info src/obstacle-avoidance-turtlebot3/results/ros_bag_file.bag
```
- To play the rosbag file (Please make sure Gazebo is not running before playing the bag file)
```
ros2 bag play src/obstacle-avoidance-turtlebot3/results/ros_bag_file.bag
```

## Running cpplint check
- Run the following command to run cpplint check
```
cpplint --filter=-build/c++11,+build/c++17,-build/name,-build/include_order,-runtime/explicit --recursive app/ src/ include/ > results/cpplint.txt  
```

## Running cppcheck 
- Run the following command to run cppceck
```
cppcheck --enable=all --std=c++17 app/ include/obstacle-avoidance-turtlebot3/ src/ --suppress=missingIncludeSystem --suppress=unmatchedSuppression --suppress=unusedFunction --suppress=missingInclude --suppress=useInitializationList > results/cppcheck.txt
```
