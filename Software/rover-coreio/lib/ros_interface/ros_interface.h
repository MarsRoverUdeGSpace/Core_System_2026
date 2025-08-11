#pragma once
#include <ros.h>

// initialize everything (baud, initNode, subs, pubs…)
void RosIfInit();

// getters for your tasks
ros::NodeHandle& RosIfGetNH();
ros::Publisher&  RosIfGetBattPub();
ros::Publisher&  RosIfGetCurrPub();

