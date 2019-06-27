/*
 * ControlManagerPlugin.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: C. Dario Bellicoso
 */

#include <pluginlib/class_list_macros.h>
#include <ros/package.h>
#include <QGridLayout>
#include <QStringList>
#include <rocoma_rqt/ControlManagerPlugin.hpp>

ControlManagerPlugin::ControlManagerPlugin() : rqt_gui_cpp::Plugin(), widget_(0), highlevelControllerNamespace_("/highlevel_controller") {
  // Constructor is called first before initPlugin function, needless to say.

  // give QObjects reasonable names
  setObjectName("ControlManagerPlugin");

  qRegisterMetaType<rocoma_msgs::SwitchControllerResponse>("rocoma_msgs::SwitchControllerResponse");
}

void ControlManagerPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  // access standalone command line arguments
  QStringList argv = context.argv();

  // create the main widget, set it up and add it to the user interface
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  context.addWidget(widget_);

  /*************************
   * Set images to buttons *
   *************************/
  ui_.pushButtonRefresh->setIcon(QIcon(QString::fromStdString(ros::package::getPath("rocoma_rqt").append("/resource/refresh.png"))));
  ui_.pushButtonRefresh->setIconSize(ui_.pushButtonRefresh->size());

  ui_.pushButtonSet->setIcon(QIcon(QString::fromStdString(ros::package::getPath("rocoma_rqt").append("/resource/set.png"))));
  ui_.pushButtonSet->setIconSize(ui_.pushButtonSet->size());

  /*************************/

  /******************************
   * Connect ui forms to actions *
   ******************************/
  connect(ui_.pushButtonEmergencyStop, SIGNAL(pressed()), this, SLOT(buttonEmergencyStopPressed()));
  connect(ui_.pushButtonRefresh, SIGNAL(pressed()), this, SLOT(buttonRefreshPressed()));
  connect(ui_.pushButtonSet, SIGNAL(pressed()), this, SLOT(buttonSetPressed()));
  connect(ui_.comboBoxActiveControllers, SIGNAL(activated(QString)), this, SLOT(comboBoxActiveControllersActivated(QString)));
  connect(ui_.pushButtonHighlvlControllerUpdate, SIGNAL(pressed()), this, SLOT(connectServiceClients()));
  /******************************/

  /******************************
   * Connect custom signals to slots*
   ******************************/

  connect(this, SIGNAL(activeControllerChanged(QString)), this, SLOT(receiveActiveControllerChanged(QString)));

  /******************************/

  // Clear labels
  ui_.labelControllerName->setText(QString::fromStdString(""));
  ui_.labelControllerStatus->setText(QString::fromStdString(""));

  // set properties for image placeholders
  ui_.labelSwitchControllerIcon->setScaledContents(true);
}

void ControlManagerPlugin::buttonEmergencyStopPressed() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;

  if (!emergencyStopClient_.call(req, res)) {
    std::cout << "[ControlManagerPlugin::buttonEmergencyStopPressed] Emergency stop failed." << std::endl;
  }

  // Clear labels
  ui_.labelControllerStatus->setText(QString::fromStdString(""));
}

void ControlManagerPlugin::buttonSetPressed() {
  rocoma_msgs::SwitchControllerRequest switchReq;
  rocoma_msgs::SwitchControllerResponse switchRes;

  switchReq.name = ui_.comboBoxActiveControllers->currentText().toStdString();
  if (controllerClient_.call(switchReq, switchRes)) {
    switch (switchRes.status) {
      case (rocoma_msgs::SwitchController::ResponseType::STATUS_RUNNING):
      case (rocoma_msgs::SwitchController::ResponseType::STATUS_SWITCHED): {
        QPixmap switchControllerImageOk(QString::fromStdString(ros::package::getPath("rocoma_rqt").append("/resource/ok.png")));
        ui_.labelSwitchControllerIcon->setPixmap(switchControllerImageOk);

      } break;

      case (rocoma_msgs::SwitchController::ResponseType::STATUS_ERROR):
      case (rocoma_msgs::SwitchController::ResponseType::STATUS_NOTFOUND): {
        QPixmap switchControllerImageError(QString::fromStdString(ros::package::getPath("rocoma_rqt").append("/resource/error.png")));
        ui_.labelSwitchControllerIcon->setPixmap(switchControllerImageError);
      } break;
    }
  } else {
    QPixmap switchControllerImageError(QString::fromStdString(ros::package::getPath("rocoma_rqt").append("/resource/error.png")));
    ui_.labelSwitchControllerIcon->setPixmap(switchControllerImageError);
  }

  ui_.labelControllerStatus->setText(QString::fromStdString(getStatusName(switchRes)));
}

void ControlManagerPlugin::buttonRefreshPressed() {
  rocoma_msgs::GetAvailableControllers::Request req;
  rocoma_msgs::GetAvailableControllers::Response res;

  ui_.comboBoxActiveControllers->clear();

  if (getAvailableControllersClient_.call(req, res)) {
    for (auto name : res.available_controllers) {
      ui_.comboBoxActiveControllers->addItem(QString::fromStdString(name));
    }
  }
}

void ControlManagerPlugin::comboBoxActiveControllersActivated(QString text) {}

std::string ControlManagerPlugin::getStatusName(rocoma_msgs::SwitchController::Response& res) {
  std::string statusName = "";
  switch (res.status) {
    case (rocoma_msgs::SwitchControllerResponse::STATUS_ERROR): {
      statusName = "STATUS_ERROR";
    } break;
    case (rocoma_msgs::SwitchControllerResponse::STATUS_NOTFOUND): {
      statusName = "STATUS_NOTFOUND";
    } break;
    case (rocoma_msgs::SwitchControllerResponse::STATUS_SWITCHED): {
      statusName = "STATUS_SWITCHED";
    } break;
    case (rocoma_msgs::SwitchControllerResponse::STATUS_RUNNING): {
      statusName = "STATUS_RUNNING";
    } break;
    default:
      statusName = "undefined";
      break;
  }

  return statusName;
}

void ControlManagerPlugin::connectServiceClients() {
  // retrieve highlevel controller namespace
  highlevelControllerNamespace_ = ui_.lineEditHighlvlControllerNamespace->text().toStdString();

  controllerClient_ =
      getNodeHandle().serviceClient<rocoma_msgs::SwitchController>(highlevelControllerNamespace_ + "/controller_manager/switch_controller");
  //  switchLowLevelControllerClient_ =
  //  getNodeHandle().serviceClient<locomotion_controller_msgs::SwitchLowLevelController>(
  //      "/lowlevel_controller/switch_controller");
  getAvailableControllersClient_ = getNodeHandle().serviceClient<rocoma_msgs::GetAvailableControllers>(
      highlevelControllerNamespace_ + "/controller_manager/get_available_controllers");
  emergencyStopClient_ =
      getNodeHandle().serviceClient<std_srvs::Trigger>(highlevelControllerNamespace_ + "/controller_manager/emergency_stop");
  getActiveControllerClient_ = getNodeHandle().serviceClient<rocoma_msgs::GetActiveController>(highlevelControllerNamespace_ +
                                                                                               "/controller_manager/get_active_controller");

  activeControllerSubscriber_ = getNodeHandle().subscribe(highlevelControllerNamespace_ + "/notify_active_controller", 1,
                                                          &ControlManagerPlugin::activeControllerCallback, this);
}

void ControlManagerPlugin::shutdownPlugin() {
  controllerClient_.shutdown();
  getAvailableControllersClient_.shutdown();
  emergencyStopClient_.shutdown();
  getActiveControllerClient_.shutdown();
}

void ControlManagerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
  MELO_INFO_STREAM("ControlManagerPlugin::saveSettings. highlevelControllerNamespace_=" << highlevelControllerNamespace_);
  instance_settings.setValue("name_space", QString::fromStdString(highlevelControllerNamespace_));
}

void ControlManagerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  // TODO restore intrinsic configuration, usually using:
  std::string highlevelControllerNamespace;

  if (getNodeHandle().getParam("highlevel_controller_ns", highlevelControllerNamespace)) {
    highlevelControllerNamespace_ = highlevelControllerNamespace;
  }
  else{
    std::string ns = getNodeHandle().getNamespace();
    MELO_WARN_STREAM("[ControlManagerPlugin] Could not read controller namespace parameter ('highlevel_controller_ns') from the server. Plugin namespace: " << ns);
    highlevelControllerNamespace_ = instance_settings.value("name_space", "/highlevel_controller").toString().toStdString();
  }
  ui_.lineEditHighlvlControllerNamespace->setText(QString::fromStdString(highlevelControllerNamespace_));

  connectServiceClients();

  buttonRefreshPressed();
}

void ControlManagerPlugin::activeControllerCallback(const std_msgs::StringConstPtr& msg) {
  std::string activeControllerName = msg->data;
  emit activeControllerChanged(QString::fromStdString(activeControllerName));
}

void ControlManagerPlugin::receiveActiveControllerChanged(QString activeControllerName) {
  ui_.labelControllerName->setText(activeControllerName);
}

/*bool hasConfiguration() const
{
  return true;
}


void triggerConfiguration()
{
  // Usually used to open a dialog to offer the user a set of configuration
}*/

PLUGINLIB_EXPORT_CLASS(ControlManagerPlugin, rqt_gui_cpp::Plugin)
