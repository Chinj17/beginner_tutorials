/**
 * @file talkerTest.cpp
 * @brief C++ file for testing nodes
 *
 * This file contains the implementation of rostest according to the
 * guidelines given in the assignment taken from ROS Wiki
 *
 * @copyright Copyright (c) Fall 2019 ENPM808X
 *            This project is released under the BSD 3-Clause License.
 *
 * @author Chinmay Joshi
 *
 * @date 11-10-2019
 */

#include "ros/ros.h"
#include "gtest/gtest.h"
#include "beginner_tutorials/changeString.h"
#include "tf/transform_listener.h"

/**
 * @brief This test function tests the service for changing the string
 */
TEST(PublisherTest, testCumstomStringService) {
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<beginner_tutorials::changeString>
  ("changeString");

  beginner_tutorials::changeString srv;
  srv.request.changeStr = "This is a default string";
  // Calling service
  client.call(srv);

  // Test to check if string has changed
  EXPECT_EQ("This is a default string", srv.request.changeStr);
}
