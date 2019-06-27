/*
 * ParametersPlugin.hpp
 *
 *  Created on: August 2016
 *      Author: Christian Gehring
 */

#pragma once

// rqt_parameters
#include <rqt_parameters/ParameterBase.hpp>
#include <rqt_parameters/ui_parameters_plugin.h>

// parameter_handler_msgs
#include <parameter_handler_msgs/GetIntegralParameter.h>
#include <parameter_handler_msgs/GetFloatingPointParameter.h>
#include <parameter_handler_msgs/SetIntegralParameter.h>
#include <parameter_handler_msgs/SetFloatingPointParameter.h>
#include <parameter_handler_msgs/GetParameterList.h>

// Qt
#include <QWidget>
#include <QDoubleSpinBox>
#include <QScrollArea>
#include <QStringList>

// Rqt
#include <rqt_gui_cpp/plugin.h>

// ros
#include <ros/ros.h>
#include <std_srvs/Empty.h>

// Stl
#include <list>
#include <memory>

namespace rqt_parameters {

class ParametersPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
public:
  ParametersPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

protected:
  virtual void shutdownServices();
  bool checkNamespace(const QString & text);

private:
  Ui::ParametersHandler ui_;
  QWidget* widget_;
  QGridLayout* paramsGrid_;
  QWidget* paramsWidget_;
  QWidget* paramsScrollHelperWidget_;
  QVBoxLayout* paramsScrollLayout_;
  QStringList namespaceList_;
  
  // ROS services
  ros::ServiceClient getParameterListClient_;
  ros::ServiceClient getIntegralParameterClient_;
  ros::ServiceClient setIntegralParameterClient_;
  ros::ServiceClient getFloatingPointParameterClient_;
  ros::ServiceClient setFloatingPointParameterClient_;
  std::string getIntegralParameterServiceName_;
  std::string setIntegralParameterServiceName_;
  std::string getFloatingPointParameterServiceName_;
  std::string setFloatingPointParameterServiceName_;
  std::string getParameterListServiceName_;

  std::list<std::shared_ptr<ParameterBase>> params_;
  std::vector<std::pair<std::string, bool>> parameterInfos_;

 protected slots:
  void refreshAll();
  void changeAll();
  void drawParamList();
  void setNamespace();

signals:
  void parametersChanged();

};

}
