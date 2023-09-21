// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros/ros.h"

#include <webots_ros/Int32Stamped.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/robot_get_device_list.h>

#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32

static double flposition = 0;
static double frposition = 0;
static double blposition = 0;
static double brposition = 0;

ros::ServiceClient frontleftWheelClient;
webots_ros::set_float frontleftWheelSrv;

ros::ServiceClient frontrightWheelClient;
webots_ros::set_float frontrightWheelSrv;

ros::ServiceClient backleftWheelClient;
webots_ros::set_float backleftWheelSrv;

ros::ServiceClient backrightWheelClient;
webots_ros::set_float backrightWheelSrv;




ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

ros::ServiceClient enableKeyboardClient;
webots_ros::set_int enableKeyboardSrv;

void quit(int sig) {
  enableKeyboardSrv.request.value = 0;
  enableKeyboardClient.call(enableKeyboardSrv);
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("User stopped the 'keyboard_teleop' node.");
  ros::shutdown();
  exit(0);
}

void keyboardCallback(const webots_ros::Int32Stamped::ConstPtr &value) {
  int key = value->data;
  int send = 0;

  switch (key) {
    case 314:
      flposition += -0.2;
      frposition += 0.2;
      blposition += -0.2;
      brposition += 0.2;
      send = 1;
      break;
    case 316:
      flposition += 0.2;
      frposition += -0.2;
      blposition += 0.2;
      brposition += -0.2;
      send = 1;
      break;
    case 315:
      flposition += 0.2;
      frposition += 0.2;
      blposition += 0.2;
      brposition += 0.2;
      send = 1;
      break;
    case 317:
      flposition += -0.2;
      frposition += -0.2;
      blposition += -0.2;
      brposition += -0.2;
      send = 1;
      break;
    case 312:
      ROS_INFO("END.");
      quit(-1);
      break;
    default:
      send = 0;
      break;
  }

  frontleftWheelSrv.request.value = flposition;
  frontrightWheelSrv.request.value = frposition;
  backleftWheelSrv.request.value = blposition;
  backrightWheelSrv.request.value = brposition;
  if (send) {
    if (!frontleftWheelClient.call(frontleftWheelSrv) || !frontrightWheelClient.call(frontrightWheelSrv) || !frontleftWheelSrv.response.success ||
        !frontrightWheelSrv.response.success)
      ROS_ERROR("Failed to send new position commands to the robot.");
  }
  return;
}

int main(int argc, char **argv) {
  // create a node named 'keyboard_teleop' on ROS network
  ros::init(argc, argv, "keyboard_teleop", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  signal(SIGINT, quit);

  // Wait for the `ros` controller.
  ros::service::waitForService("/pioneer3at/robot/time_step");
  ros::spinOnce();

  frontleftWheelClient = n.serviceClient<webots_ros::set_float>("/pioneer3at/front_left_wheel/set_position");
  frontrightWheelClient = n.serviceClient<webots_ros::set_float>("/pioneer3at/front_right_wheel/set_position");
  backleftWheelClient = n.serviceClient<webots_ros::set_float>("/pioneer3at/back_left_wheel/set_position");
  backrightWheelClient = n.serviceClient<webots_ros::set_float>("/pioneer3at/back_right_wheel/set_position");
  timeStepClient = n.serviceClient<webots_ros::set_int>("/pioneer3at/robot/time_step");

  timeStepSrv.request.value = TIME_STEP;

  enableKeyboardClient = n.serviceClient<webots_ros::set_int>("/keyboard/enable");
  enableKeyboardSrv.request.value = TIME_STEP;
  if (enableKeyboardClient.call(enableKeyboardSrv) && enableKeyboardSrv.response.success) {
    ros::Subscriber sub_keyboard;
    sub_keyboard = n.subscribe("/keyboard/key", 1, keyboardCallback);
    while (sub_keyboard.getNumPublishers() == 0) {
    }
    ROS_INFO("Keyboard enabled.");
    ROS_INFO("Use the arrows in Webots window to move the robot.");
    ROS_INFO("Press the End key to stop the node.");

    // main loop
    while (ros::ok()) {
      ros::spinOnce();
      if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");
    }
  } else
    ROS_ERROR("Could not enable keyboard, success = %d.", enableKeyboardSrv.response.success);

  enableKeyboardSrv.request.value = 0;
  if (!enableKeyboardClient.call(enableKeyboardSrv) || !enableKeyboardSrv.response.success)
    ROS_ERROR("Could not disable keyboard, success = %d.", enableKeyboardSrv.response.success);
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  return (0);
}
