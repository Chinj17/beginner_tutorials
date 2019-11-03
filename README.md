# ROS Publisher/Subscriber
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
This program is an example of a basic publisher/subscriber in ROS. I have created a catkin package, edited the package.xml and created two nodes talker.cpp and listener.cpp.

# Software
This program is running on a device running Ubuntu 16.04 and ROS Kinetic.
* To install ROS kinetic, use this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* If you don't have catkin installed use this [link](http://wiki.ros.org/catkin)

# Build and Compile
This code has to run in a catkin workspace. If you don't have a catkin workspace created use the following command to create one:
```
mkdir -p ~/catkin_ws/src
```
If you already have a catkin workspace or have created one using the above line then:
```
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/Chinj17/beginner_tutorials.git
cd ..
catkin_make
source devel/setup.bash
```

# Running the code
You can either launch all the files at once or call them individually
1. Using ROS launch
* Using default frequency of 10Hz
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials subPubNodes.launch
```
* Using user defined frequency
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch beginner_tutorials subPubNodes.launch frequency:=15
```
2. Running each node individually
* A ROS master has to be Running on one terminal
```
cd ~catkin_ws/
source devel/setup.bash
roscore
```
* Publisher node on the second terminal
```
cd ~catkin_ws/
source devel/setup.bash
rosrun beginner_tutorials talker
```
* Subscriber node on the third terminal
```
cd ~catkin_ws/
source devel/setup.bash
rosrun beginner_tutorials listener
```
3. Running ROS service to change the string input
* In a separate terminal do the following
```
cd ~catkin_ws/
source devel/setup.bash
rosservice call /customString "<user defined string>"
```
# Error checks

**cppcheck**
```
cd <path to directory>
cppcheck --std=c++11 $(find . -name \*.cpp -or -name \*.srv | grep -vE -e "^./build/" -e "^./results/") &> cppcheck.txt
```
**Google C++ standards**
```
cd <path to directory>
cpplint $(find . -name \*.cpp | grep -vE -e "^./build/" -e "^./results") &> cpplint.txt
```
