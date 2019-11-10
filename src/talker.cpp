/**
 * @file talker.cpp
 * @brief C++ file for the publisher node
 *
 * This file contains the implementation of tutorials according to the
 * guidelines given in the assignment taken from ROS Wiki
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi
 *
 * @date 11-10-2019
 */
#include <sstream>
#include <memory>
#include <string>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/changeString.h"
#include "tf/transform_broadcaster.h"

// Creating a smart pointer as per C++11 standards
std::unique_ptr<std::string> stringPointer (new std::string);

/**
 * @brief Callback function that enables changing message of publisher
 * @param Parameter 1, request to service to change message
 * @param Parameter 2, response to service to change message
 * @return boolean value, true if message change sent
 */
bool customString(beginner_tutorials::changeString::Request &req,
beginner_tutorials::changeString::Response &res) {
    res.changeStr = req.changeStr;
    // Resetting the pointer to custom string given by user through service
    stringPointer.reset(new std::string);
    *stringPointer = res.changeStr;
    ROS_INFO_STREAM("Sending modified response: ");
    return true;
  }

/**
 * @brief This tutorial demonstrates simple sending of messages
 * over the ROS system.
 * @param argc is the number of inputs
 * @param argv is the input
 * @return integer 0 upon successful execution
 */

int main(int argc, char **argv) {
  std::string original = "It is time for a change! ";
  // Assigning the pointer to default string
  stringPointer.reset(new std::string);
  *stringPointer = original;
/**
 * The ros::init() function needs to see argc and argv so that it can perform
 * any ROS arguments and name remapping that were provided at the command line.
 * For programmatic remappings you can use a different version of init() which takes
 * remappings directly, but for most command-line programs, passing argc and argv is
 * the easiest way to do it.  The third argument to init() is the name of the node.
 *
 * You must call one of the versions of ros::init() before using any other
 * part of the ROS system.
 */
  ros::init(argc, argv, "talker");
 /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
  ros::NodeHandle n;

  ros::ServiceServer srvString = n.advertiseService("customString",
                                                     customString);

 /**
  * The advertise() function is how you tell ROS that you want to
  * publish on a given topic name. This invokes a call to the ROS
  * master node, which keeps a registry of who is publishing and who
  * is subscribing. After this advertise() call is made, the master
  * node will notify anyone who is trying to subscribe to this topic name,
  * and they will in turn negotiate a peer-to-peer connection with this
  * node.  advertise() returns a Publisher object which allows you to
  * publish messages on that topic through a call to publish().  Once
  * all copies of the returned Publisher object are destroyed, the topic
  * will be automatically unadvertised.
  *
  * The second parameter to advertise() is the size of the message queue
  * used for publishing messages.  If messages are published more quickly
  * than we can send them, the number here specifies how many messages to
  * buffer up before throwing some away.
  */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  // Setting default frequency to 10Hz
  int frequency = 10;
  // Checking for more than 1 input
  if (argc > 1) {
    // Converting the string to integer
    frequency = std::stoi(argv[1]);
    // Giving a warning if frequency is less than 5Hz
    if (frequency < 5) {
      ROS_WARN_STREAM("Entered frequency is low, shift to a higher frequency");
    }
    // Giving a fatal error if frequency 0 or negative
    if (frequency <= 0) {
      ROS_FATAL_STREAM("Frequency cannot be zero or negative");
    }
  }

  ros::Rate loop_rate(frequency);
  ROS_INFO_STREAM("Frequency has been set! ");
 /**
  * A count of how many messages we have sent. This is used to create
  * a unique string for each message.
  */

  int count = 0;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;

  while (ros::ok()) {
 /**
  * This is a message object. You stuff it with data, and then publish it.
  */
  std_msgs::String msg;

  std::stringstream ss;

  if (*stringPointer == "") {
    ROS_ERROR_STREAM("Empty string");
  }

  ss << *stringPointer << count;
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());
 /**
  * The publish() function is how you send messages. The parameter
  * is the message object. The type of this object must agree with the type
  * given as a template parameter to the advertise<>() call, as was done
  * in the constructor above.
  */
  chatter_pub.publish(msg);
  transform.setOrigin(tf::Vector3(sin(count), cos(count), 1));
  q.setRPY(3.14, 3.14/2, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                  "world", "talk"));

  ros::spinOnce();
  loop_rate.sleep();
  ++count;
  }
  return 0;
}
