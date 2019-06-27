/*!
* @file     JoyManagerTest.cpp
* @author   Linus Isler
* @date     June, 2016
* @brief
*/

#include <iterator>
#include <unordered_map>

#include <gtest/gtest.h>
#include <ros/ros.h>

#include <joy_manager/JoyManager.hpp>
#include <joy_manager_msgs/AnyJoy.h>
#include <notification_msgs/Notification.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Empty.h>



using namespace joy_manager;

class JoyCallback {
public:
  JoyCallback()
  : firstEntry_(0.0) {
  };
  void callback(const sensor_msgs::JoyConstPtr& msg) {
    if (!msg->axes.empty() && msg->axes[0] != 0.0) {
      firstEntry_ = msg->axes[0];
    }
  }
  double getEntry() {
    return firstEntry_;
  }
private:
  double firstEntry_;
};


/* Test the joy manager by sending one AnyJoy message from each joystick
 * and test if the highest prioritiesed message is published further.
 */
TEST(JoyManager, Priorities)
{
  ros::NodeHandle nh("~");
  notification_msgs::NotificationConstPtr receiveNotification =
      ros::topic::waitForMessage<notification_msgs::Notification>("/notification", nh, ros::Duration(5.0));
  EXPECT_EQ(receiveNotification->name, "JM start up");

  // initialize publishers
  std::unordered_map<int, ros::Publisher> joysticks;
  int highestPriority = 0;
  XmlRpc::XmlRpcValue params;
  nh.getParam("/joy_manager/joysticks", params);
  for (int i = 0; i < params.size(); i++) {
    XmlRpc::XmlRpcValue param = params[i];
    std::string joystickTopic = static_cast<std::string>(param["topic"]);
    int joystickPriority = static_cast<int>(param["priority"]);
    if (joystickPriority > highestPriority)
      highestPriority = joystickPriority;
    ros::Publisher joystickPublisher = nh.advertise<joy_manager_msgs::AnyJoy>(joystickTopic, 1);
    joysticks.insert((std::pair<int, ros::Publisher>(joystickPriority, joystickPublisher)));
  }
  if (joysticks.size() < 2) {
    ROS_ERROR("Please specify more than one Joystick in the config file!");
    FAIL();
  }

  // send messages
  joy_manager_msgs::AnyJoy sendAnyJoyMsg;
  sendAnyJoyMsg.header.stamp = ros::Time::now();
  sendAnyJoyMsg.joy.axes.assign(8, 0.0);
  sendAnyJoyMsg.joy.buttons.assign(8, 0);
  for (std::pair<int, ros::Publisher> joystick : joysticks) {
    ros::Duration(0.5).sleep();
    sendAnyJoyMsg.joy.axes[1] = joystick.first;
    joystick.second.publish(sendAnyJoyMsg);
  }

  // receive message
  sensor_msgs::JoyConstPtr receiveJoyMsg =
      ros::topic::waitForMessage<sensor_msgs::Joy>("/joy_manager/joy", nh, ros::Duration(2.0));
  EXPECT_EQ(highestPriority, receiveJoyMsg->axes[1]);
}


/* Test the joy manager by sending one AnyJoy message with high priority,
 * one while the timer still runs with lower priority and one after the timer expires
 * one with lower priority.
 */
TEST(JoyManager, Timer)
{
  ros::NodeHandle nh("~");
  double timerMax;
  nh.getParam("/joy_manager/timer", timerMax);

  // initialize publishers
  std::unordered_map<int, ros::Publisher> joysticks;
  int highestPriority = 0;
  XmlRpc::XmlRpcValue params;
  nh.getParam("/joy_manager/joysticks", params);
  for (int i = 0; i < params.size(); i++) {
    XmlRpc::XmlRpcValue param = params[i];
    std::string joystickTopic = static_cast<std::string>(param["topic"]);
    int joystickPriority = static_cast<int>(param["priority"]);
    if (joystickPriority > highestPriority)
      highestPriority = joystickPriority;
    ros::Publisher joystickPublisher = nh.advertise<joy_manager_msgs::AnyJoy>(joystickTopic, 1);
    joysticks.insert((std::pair<int, ros::Publisher>(joystickPriority, joystickPublisher)));
  }
  if (joysticks.size() < 2) {
    ROS_ERROR("Please specify more than one Joystick in the config file!");
    return;
  }

  // sending first message
  std::unordered_map<int, ros::Publisher>::const_iterator highestJoystick =
      joysticks.find(highestPriority);
  joy_manager_msgs::AnyJoy sendAnyJoyMsg;
  sendAnyJoyMsg.joy.axes.assign(8, 0.0);
  sendAnyJoyMsg.joy.buttons.assign(8, 0);
  sendAnyJoyMsg.joy.axes[0] = 3;
  ros::Duration(0.5).sleep();
  sendAnyJoyMsg.header.stamp = ros::Time::now();
  ros::Time twoSecs = ros::Time::now() + ros::Duration(2.0);
  JoyCallback joyCallback;
  ros::Subscriber joySub = nh.subscribe("/joy_manager/joy", 10, &JoyCallback::callback, &joyCallback);
  while(ros::Time::now() < twoSecs) {
    highestJoystick->second.publish(sendAnyJoyMsg);
    ros::spinOnce();
    ros::Duration(0.02).sleep();
  }
  EXPECT_EQ(joyCallback.getEntry(), 3);
  ros::Duration(timerMax/2.0).sleep();

  // sending second message
  for (std::pair<int, ros::Publisher> joystick : joysticks) {
    if (joystick.first < highestPriority) {
      sendAnyJoyMsg.joy.axes[0] = 2;
      ros::Duration(0.5).sleep();
      sendAnyJoyMsg.header.stamp = ros::Time::now();
      twoSecs = ros::Time::now() + ros::Duration(2.0);
      while(ros::Time::now() < twoSecs) {
        joystick.second.publish(sendAnyJoyMsg);
        ros::spinOnce();
        ros::Duration(0.02).sleep();
      }
      EXPECT_EQ(joyCallback.getEntry(), 3);
      break;
    }
  }

//  sensor_msgs::JoyConstPtr receiveJoyMsg2 =
//      ros::topic::waitForMessage<sensor_msgs::Joy>("/commands/joy", nh, ros::Duration(2.0));
//  EXPECT_EQ(receiveJoyMsg2->axes[0], 3);
  ros::Duration(timerMax/2.0 + 0.5).sleep();

  // sending third message
  for (std::pair<int, ros::Publisher> joystick : joysticks) {
    if (joystick.first < highestPriority) {
      sendAnyJoyMsg.joy.axes[0] = 1;
      ros::Duration(0.5).sleep();
      sendAnyJoyMsg.header.stamp = ros::Time::now();
      twoSecs = ros::Time::now() + ros::Duration(2.0);
      while(ros::Time::now() < twoSecs) {
        highestJoystick->second.publish(sendAnyJoyMsg);
        ros::spinOnce();
        ros::Duration(0.02).sleep();
      }
      EXPECT_EQ(joyCallback.getEntry(), 1);
      break;
    }
  }
//  sensor_msgs::JoyConstPtr receiveJoyMsg3 =
//      ros::topic::waitForMessage<sensor_msgs::Joy>("/commands/joy", nh, ros::Duration(2.0));
//  EXPECT_EQ(receiveJoyMsg3->axes[0], 1);
}



/* Test the JoyManager by sending unspecified commands to modules.
 * Expected is the transmission of an error notification message
 * on "/notification".
 */
TEST(JoyManager, Modules)
{
  ros::NodeHandle nh("~");

  // initialize publishers
  std::unordered_map<int, ros::Publisher> joysticks;
  int highestPriority = 0;
  XmlRpc::XmlRpcValue params;
  nh.getParam("/joy_manager/joysticks", params);
  for (int i = 0; i < params.size(); i++) {
    XmlRpc::XmlRpcValue param = params[i];
    std::string joystickTopic = static_cast<std::string>(param["topic"]);
    int joystickPriority = static_cast<int>(param["priority"]);
    if (joystickPriority > highestPriority)
      highestPriority = joystickPriority;
    ros::Publisher joystickPublisher = nh.advertise<joy_manager_msgs::AnyJoy>(joystickTopic, 1);
    joysticks.insert((std::pair<int, ros::Publisher>(joystickPriority, joystickPublisher)));
  }
  if (joysticks.empty()) {
    ROS_ERROR("Please load at least one joystick in the JoyManager config file!");
    FAIL();
  }

  // send unknown commands to unknown module
  std::unordered_map<int, ros::Publisher>::const_iterator highestJoystick =
        joysticks.find(highestPriority);
  joy_manager_msgs::AnyJoy sendAnyJoyMsg;
  sendAnyJoyMsg.header.stamp = ros::Time::now();
  sendAnyJoyMsg.joy.axes.assign(8, 0.0);
  sendAnyJoyMsg.joy.buttons.assign(8, 0);
  sendAnyJoyMsg.modules.push_back("unspecifiedModule");
  sendAnyJoyMsg.commands.push_back("unspecifiedCommand");
  ros::Duration(0.5).sleep();
  notification_msgs::NotificationConstPtr receiveNotification;
  highestJoystick->second.publish(sendAnyJoyMsg);
  receiveNotification = ros::topic::waitForMessage<notification_msgs::Notification>("/notification", nh, ros::Duration(2.0));
  if (receiveNotification == NULL) {
    FAIL();
  }
  EXPECT_EQ(receiveNotification->name, "unknown module");

  // send unknown commands to known module
  std::vector<std::string> modules;
  nh.getParam("/joy_manager/modules", modules);
  if (modules.empty()) {
    ROS_ERROR("Please load at least one Module in the JoyManager config file!");
    FAIL();
  }
  std::string randModule = modules.at(rand() % modules.size());
  sendAnyJoyMsg.modules.at(0) = randModule;
  ROS_ERROR_STREAM("module 0 = " << randModule);
  sendAnyJoyMsg.commands.at(0) = "unspecifiedCommand";
  ros::Duration(0.5).sleep();
  sendAnyJoyMsg.header.stamp = ros::Time::now();
  highestJoystick->second.publish(sendAnyJoyMsg);
  notification_msgs::NotificationConstPtr receiveNotification2Msg =
      ros::topic::waitForMessage<notification_msgs::Notification>("/notification", nh, ros::Duration(2.0));
  if (receiveNotification2Msg == NULL) {
      FAIL();
  }
  EXPECT_TRUE(receiveNotification2Msg->name != "success");
}




/* Test the JoyManager by sending the "soft_emcy_stop" command from a
 * randomly picked joystick. Expected is the transmission of an empty
 * message on "/soft_emcy_stop".
 */
TEST(JoyManager, SoftEmergencyForwarding)
{
  ros::NodeHandle nh("~");

  // initialize publishers
  std::unordered_map<int, ros::Publisher> joysticks;
  XmlRpc::XmlRpcValue params;
  nh.getParam("/joy_manager/joysticks", params);
  for (int i = 0; i < params.size(); i++) {
    XmlRpc::XmlRpcValue param = params[i];
    std::string joystickTopic = static_cast<std::string>(param["topic"]);
    int joystickPriority = static_cast<int>(param["priority"]);
    ros::Publisher joystickPublisher = nh.advertise<joy_manager_msgs::AnyJoy>(joystickTopic, 1);
    joysticks.insert((std::pair<int, ros::Publisher>(joystickPriority, joystickPublisher)));
  }
  if (joysticks.empty()) {
    ROS_ERROR("Please load at least one joystick in the JoyManager config file!");
    FAIL();
  }

  // send messages from all joysticks
  joy_manager_msgs::AnyJoy sendAnyJoyMsg;
  sendAnyJoyMsg.header.stamp = ros::Time::now();
  sendAnyJoyMsg.joy.axes.assign(8, 0.0);
  sendAnyJoyMsg.joy.buttons.assign(8, 0);
  for (std::pair<int, ros::Publisher> joystick : joysticks) {
    ros::Duration(0.5).sleep();
    sendAnyJoyMsg.joy.axes[1] = joystick.first;
    joystick.second.publish(sendAnyJoyMsg);
  }

  // sending emergency message
  std::unordered_map<int, ros::Publisher>::iterator joystick = joysticks.begin();
  std::advance(joystick, rand() % joysticks.size());
  sendAnyJoyMsg.joy.axes.assign(8, 0.0);
  sendAnyJoyMsg.joy.buttons.assign(8, 0);
  sendAnyJoyMsg.commands = {"soft_emcy_stop"};
  ros::Duration(0.5).sleep();
  sendAnyJoyMsg.header.stamp = ros::Time::now();
  joystick->second.publish(sendAnyJoyMsg);
  std_msgs::EmptyConstPtr receiveEmptyMsg =
      ros::topic::waitForMessage<std_msgs::Empty>("/soft_emcy_stop", nh, ros::Duration(1.0));
  SUCCEED();
}


/* Test the JoyManager by sending the "hard_emcy_stop" command from a
 * randomly picked joystick. Expected is the transmission of an empty
 * message on "/hard_emcy_stop".
 */
TEST(JoyManager, HardEmergencyForwarding)
{
  ros::NodeHandle nh("~");

  // initialize publishers
  std::unordered_map<int, ros::Publisher> joysticks;
  XmlRpc::XmlRpcValue params;
  nh.getParam("/joy_manager/joysticks", params);
  for (int i = 0; i < params.size(); i++) {
    XmlRpc::XmlRpcValue param = params[i];
    std::string joystickTopic = static_cast<std::string>(param["topic"]);
    int joystickPriority = static_cast<int>(param["priority"]);
    ros::Publisher joystickPublisher = nh.advertise<joy_manager_msgs::AnyJoy>(joystickTopic, 1);
    joysticks.insert((std::pair<int, ros::Publisher>(joystickPriority, joystickPublisher)));
  }
  if (joysticks.empty()) {
    ROS_ERROR("Please load at least one joystick in the JoyManager config file!");
    FAIL();
  }

  // send messages from all joysticks
  joy_manager_msgs::AnyJoy sendAnyJoyMsg;
  sendAnyJoyMsg.header.stamp = ros::Time::now();
  sendAnyJoyMsg.joy.axes.assign(8, 0.0);
  sendAnyJoyMsg.joy.buttons.assign(8, 0);
  for (std::pair<int, ros::Publisher> joystick : joysticks) {
    ros::Duration(0.5).sleep();
    sendAnyJoyMsg.joy.axes[1] = joystick.first;
    joystick.second.publish(sendAnyJoyMsg);
  }

  // sending emergency message
  std::unordered_map<int, ros::Publisher>::iterator randJoystick = joysticks.begin();
  std::advance(randJoystick, rand() % joysticks.size());
  sendAnyJoyMsg.joy.axes.assign(8, 0.0);
  sendAnyJoyMsg.joy.buttons.assign(8, 0);
  sendAnyJoyMsg.commands = {"hard_emcy_stop"};
  ros::Duration(0.5).sleep();
  sendAnyJoyMsg.header.stamp = ros::Time::now();
  randJoystick->second.publish(sendAnyJoyMsg);
  std_msgs::EmptyConstPtr receiveEmptyMsg =
      ros::topic::waitForMessage<std_msgs::Empty>("/hard_emcy_stop", nh, ros::Duration(1.0));
  SUCCEED();
}
