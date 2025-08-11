#include "ros_interface.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Joy.h>
#include "hal_led.h"
#include "hal_battery.h"
#include "hal_motor.h"

static ros::NodeHandle nh;
static std_msgs::Float32 batt_msg;
static std_msgs::Float32 curr_msg;
static ros::Publisher batt_pub("battery_voltage",&batt_msg);
static ros::Publisher curr_pub("motor_current", &curr_msg);

// ROS callbacks
static void ledCb(const std_msgs::Int8 &m) {
  HalLedHandleMode(static_cast<OperationMode>(m.data));
}

static void joyCb(const sensor_msgs::Joy &joy) {
  MotorCommand_t cmd = HalMotorMakeCommand(joy);
  HalMotorQueueCommand(&cmd);
}

void RosIfInit() {
  nh.getHardware()->setBaud(115200);
  nh.initNode();

  static ros::Subscriber<std_msgs::Int8> sub_mode("onboardLED", ledCb);
  static ros::Subscriber<sensor_msgs::Joy> sub_joy("joy", joyCb);

  nh.subscribe(sub_mode);
  nh.subscribe(sub_joy);
  nh.advertise(batt_pub);
  nh.advertise(curr_pub);
}

ros::NodeHandle& RosIfGetNH()       { return nh; }
ros::Publisher&  RosIfGetBattPub()  { return batt_pub; }
ros::Publisher&  RosIfGetCurrPub()  { return curr_pub; }

