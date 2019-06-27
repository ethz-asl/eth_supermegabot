/*
 * GazeboInterfacePlugin.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
 */


#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QGridLayout>
#include <std_srvs/Empty.h>

#include "rqt_gazebo_interface/GazeboInterfacePlugin.hpp"


GazeboInterfacePlugin::GazeboInterfacePlugin() :
    rqt_gui_cpp::Plugin()
{
  // give QObjects reasonable names
  setObjectName("GazeboInterfacePlugin");
}


void GazeboInterfacePlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
    // access standalone command line arguments
    QStringList argv = context.argv();

    // create the main widget, set it up and add it to the user interface
    widget_ = new QWidget();
    ui_.setupUi(widget_);
    context.addWidget(widget_);

    connect(ui_.pushButtonPausePlay, SIGNAL(pressed()),this,SLOT(buttonPausePlayPressed()));
    connect(ui_.pushButtonReset, SIGNAL(pressed()),this,SLOT(buttonResetPressed()));
    connect(ui_.pushButtonSky, SIGNAL(pressed()),this,SLOT(buttonSkyPressed()));
    connect(ui_.pushButtonThrowBall, SIGNAL(pressed()),this,SLOT(buttonThrowBallPressed()));

    connect(ui_.sliderSimulationTimeRate, SIGNAL(valueChanged(int)), this, SLOT(sliderSimulationTimeMoved()));
    connect(ui_.lineEditSetSimTime, SIGNAL(editingFinished()), this, SLOT(lineEditSetSimTimeEditingFinished()));

    gazeboSrvGetPhysics_ = getNodeHandle().serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
    gazeboSrvSetPhysics_ = getNodeHandle().serviceClient<gazebo_msgs::SetPhysicsProperties>("/gazebo/set_physics_properties");

    gazeboSrvPause_ = getNodeHandle().serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    gazeboSrvUnPause_ = getNodeHandle().serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    gazeboSrvReset_ = getNodeHandle().serviceClient<std_srvs::Empty>("/gazebo/reset_world");

    throwBallServiceClient_ = getNodeHandle().serviceClient<std_srvs::Empty>("/dodgeball/throw");

   if (gazeboSrvGetPhysics_.call(gazGetPhysics_)) {
       isPaused_ = gazGetPhysics_.response.pause;
   }

}


void GazeboInterfacePlugin::copyResponseToRequest(const gazebo_msgs::GetPhysicsProperties& getPhys, gazebo_msgs::SetPhysicsProperties& setPhys) {
    setPhys.request.gravity         = getPhys.response.gravity;
    setPhys.request.max_update_rate = getPhys.response.max_update_rate;
    setPhys.request.ode_config      = getPhys.response.ode_config;
    setPhys.request.time_step       = getPhys.response.time_step;
}


void GazeboInterfacePlugin::lineEditSetSimTimeEditingFinished() {
    int desiredSimTime = std::max(0, 10*ui_.lineEditSetSimTime->text().toInt());
    int desiredSimTimeClamped = std::min( ui_.sliderSimulationTimeRate->maximum(), std::max( ui_.sliderSimulationTimeRate->minimum(), desiredSimTime ) );
    ui_.sliderSimulationTimeRate->setValue(desiredSimTimeClamped);

    // reapply value to lineEdit
    ui_.lineEditSetSimTime->setText(QString::number(desiredSimTime/10));

    if (gazeboSrvGetPhysics_.call(gazGetPhysics_)) {
        copyResponseToRequest(gazGetPhysics_, gazSetPhysics_);
        gazSetPhysics_.request.max_update_rate = desiredSimTime;
        gazeboSrvSetPhysics_.call(gazSetPhysics_);
    }
}


void GazeboInterfacePlugin::buttonSkyPressed() {
    gazeboSrvGetPhysics_.call(gazGetPhysics_);
    copyResponseToRequest(gazGetPhysics_, gazSetPhysics_);
    if (hasNormalGravity_) {
      gazSetPhysics_.request.gravity.z = -0.1;
      hasNormalGravity_ = false;
    } else {
      gazSetPhysics_.request.gravity.z = -9.81;
      hasNormalGravity_ = true;
    }
    gazeboSrvSetPhysics_.call(gazSetPhysics_);
}

void GazeboInterfacePlugin::sliderSimulationTimeMoved() {
    ui_.lineEditSetSimTime->setText(QString::number(ui_.sliderSimulationTimeRate->value()/10));

    if (gazeboSrvGetPhysics_.call(gazGetPhysics_)) {
        copyResponseToRequest(gazGetPhysics_, gazSetPhysics_);
        gazSetPhysics_.request.max_update_rate = (double)ui_.sliderSimulationTimeRate->value();

        gazeboSrvSetPhysics_.call(gazSetPhysics_);
    }
}


void GazeboInterfacePlugin::buttonPausePlayPressed() {
    std_srvs::Empty emptyService;

    if (isPaused_) {
        if (gazeboSrvUnPause_.call(emptyService)) {
            isPaused_ = false;
        }
    } else {
        if (gazeboSrvPause_.call(emptyService)) {
            isPaused_ = true;
        }
    }
}


void GazeboInterfacePlugin::buttonResetPressed() {
    std_srvs::Empty emptyService;
    gazeboSrvReset_.call(emptyService);
}

void GazeboInterfacePlugin::buttonThrowBallPressed() {
    std_srvs::Empty emptyService;
    throwBallServiceClient_.call(emptyService);
}


void GazeboInterfacePlugin::shutdownPlugin() {
    gazeboSrvGetPhysics_.shutdown();
    gazeboSrvSetPhysics_.shutdown();

    gazeboSrvPause_.shutdown();
    gazeboSrvUnPause_.shutdown();
    gazeboSrvReset_.shutdown();

    throwBallServiceClient_.shutdown();
}


void GazeboInterfacePlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}


void GazeboInterfacePlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}


/*bool hasConfiguration() const
{
  return true;
}


void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/


PLUGINLIB_EXPORT_CLASS(GazeboInterfacePlugin, rqt_gui_cpp::Plugin)
