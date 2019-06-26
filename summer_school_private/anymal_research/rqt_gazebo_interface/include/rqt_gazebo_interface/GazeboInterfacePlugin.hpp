/*
 * GazeboInterfacePlugin.hpp
 *
 *  Created on: Jan 21, 2015
 *      Author: C. Dario Bellicoso
 */

#ifndef GAZEBOINTERFACEPLUGIN_HPP
#define GAZEBOINTERFACEPLUGIN_HPP


#include <rqt_gui_cpp/plugin.h>
#include <ui_gazebo_interface_plugin.h>
#include <QWidget>

#include <ros/ros.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_srvs/Empty.h>

class GazeboInterfacePlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
public:
    GazeboInterfacePlugin();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

    // Comment in to signal that the plugin has a way to configure it
    //bool hasConfiguration() const;
    //void triggerConfiguration();
private:
    Ui::GazeboInterface ui_;
    QWidget* widget_ = nullptr;
    ros::ServiceClient gazeboSrvGetPhysics_;
    ros::ServiceClient gazeboSrvSetPhysics_;

    ros::ServiceClient gazeboSrvPause_;
    ros::ServiceClient gazeboSrvUnPause_;
    ros::ServiceClient gazeboSrvReset_;

    ros::ServiceClient throwBallServiceClient_;

    void copyResponseToRequest(const gazebo_msgs::GetPhysicsProperties& getPhys, gazebo_msgs::SetPhysicsProperties& setPhys);
    gazebo_msgs::GetPhysicsProperties gazGetPhysics_;
    gazebo_msgs::SetPhysicsProperties gazSetPhysics_;

    double sliderBeforePause_ = 1000.0;
    bool isPaused_ = false;
    bool hasNormalGravity_ = true;

protected slots:
    virtual void buttonPausePlayPressed();
    virtual void buttonResetPressed();
    virtual void buttonSkyPressed();
    virtual void buttonThrowBallPressed();

    virtual void lineEditSetSimTimeEditingFinished();
    virtual void sliderSimulationTimeMoved();

signals:
    void stateChanged();
};

#endif // GAZEBOINTERFACEPLUGIN_HPP

