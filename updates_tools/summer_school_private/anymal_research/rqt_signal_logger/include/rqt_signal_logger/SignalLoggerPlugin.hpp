/*!
* @file     SignalLoggerPlugin.hpp
* @author   Gabriel Hottiger, Samuel Bachmann
* @date     October 10, 2016
* @brief    Signal Logger Rqt plugin.
*/

#pragma once

// rqt_signal_logger
#include "rqt_signal_logger/LogElement.hpp"

// custom msgs/srvs
#include "signal_logger_msgs/EditLoggerScript.h"
#include "signal_logger_msgs/SetLoggerElement.h"
#include "signal_logger_msgs/GetLoggerElement.h"
#include "signal_logger_msgs/GetLoggerConfiguration.h"
#include "signal_logger_msgs/SaveLoggerData.h"

// ros
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>

// QT
#include <rqt_gui_cpp/plugin.h>
#include <ui_SignalLoggerPlugin.h>
#include <ui_SignalLoggerConfigure.h>
#include <ui_SignalLoggerVariables.h>
#include <QWidget>
#include <QTabWidget>
#include <QDoubleSpinBox>
#include <QScrollArea>
#include <QStatusBar>

// STL
#include <list>
#include <memory>

namespace rqt_signal_logger {

class SignalLoggerPlugin : public rqt_gui_cpp::Plugin {
  Q_OBJECT
 public:
  //! Status message types
  enum class MessageType {
    ERROR,
    WARNING,
    SUCCESS,
    STATUS
  };
  //! Tasks in combo box
  enum class TaskList {
    ENABLE_ALL,
    DISABLE_ALL,
    SET_DIVIDER,
    SET_ACTION,
    SET_BUFFER_TYPE,
    SET_BUFFER_SIZE,
    SET_BUFFER_SIZE_FROM_TIME
  };

 public:
  SignalLoggerPlugin();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
  void statusMessage(std::string message, MessageType type, double displaySeconds = 2.0);
  void shutdownROS();
  bool checkNamespace(const QString & text);

  protected slots:
    void refreshAll();
    void changeAll();
    void drawParamList();
    void setNamespace();
    void startLogger();
    void stopLogger();
    void restartLogger();
    void saveLoggerData();
    void selectYamlFile();
    void loadYamlFile();
    void saveYamlFile();
    void taskChanged(int index);
    void valueChanged(double value);
    void applyButtonPressed();
    void checkLoggerState();

  signals:
    void parametersChanged();

 private:
  Ui::SignalLoggerPlugin ui_;
  Ui::SignalLoggerVariables varsUi_;
  Ui::SignalLoggerConfigure configureUi_;
  QWidget* widget_;
  QTabWidget* tabWidget_;
  QStatusBar* statusBar_;
  QWidget* varsWidget_;
  QWidget* configureWidget_;
  QGridLayout* paramsGrid_;
  QWidget* paramsWidget_;
  QWidget* paramsScrollHelperWidget_;
  QVBoxLayout* paramsScrollLayout_;
  QStringList namespaceList_;


  // ROS services
  ros::ServiceClient getLoggerConfigurationClient_;
  ros::ServiceClient getLoggerElementClient_;
  ros::ServiceClient setLoggerElementClient_;
  ros::ServiceClient startLoggerClient_;
  ros::ServiceClient stopLoggerClient_;
  ros::ServiceClient saveLoggerDataClient_;
  ros::ServiceClient loadLoggerScriptClient_;
  ros::ServiceClient saveLoggerScriptClient_;
  ros::ServiceClient isLoggerRunningClient_;

  std::string getLoggerConfigurationServiceName_;
  std::string getParameterServiceName_;
  std::string setParameterServiceName_;
  std::string startLoggerServiceName_;
  std::string stopLoggerServiceName_;
  std::string saveLoggerDataServiceName_;
  std::string loadLoggerScriptServiceName_;
  std::string saveLoggerScriptServiceName_;
  std::string isLoggerRunningServiceName_;

  std::vector<std::shared_ptr<LogElement>> logElements_;
  std::vector<std::string> logElementNames_;
  double updateFrequency_;
};


}
