# beginner_tutorials
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Overview
This program is an example of a basic publisher/subscriber in ROS. I have created a catkin package, edited the package.xml and created two nodes talker.cpp and listener.cpp.

# Software
This program is running on a device running Ubuntu 16.04 and ROS Kinetic.
* To install ROS kinetic, use this [link](http://wiki.ros.org/kinetic/Installation/Ubuntu)

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
# Error checks

**cppcheck**
```
cd <path to directory>
cppcheck --std=c++11 $(find . -name \*.cpp -or -name \*.srv | grep -vE -e "^./build/" -e "^./results/")
```
**Google C++ standards**
```
cd <path to directory>
cpplint $(find . -name \*.cpp | grep -vE -e "^./build/" -e "^./results")
```
