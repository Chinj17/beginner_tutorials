/**
 * @file talker.cpp
 * @brief main file for the rostests
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

#include <gtest/gtest.h>
#include <ros/ros.h>

/**
* @brief main function which runs all test results.
*/
int main(int argc, char** argv) {
  // Initialize ROS node
  ros::init(argc, argv, "beginnerTutorialsTest");

  // Initialize Google Test
  ::testing::InitGoogleTest(&argc, argv);

  // Run all tests
  return RUN_ALL_TESTS();
}
