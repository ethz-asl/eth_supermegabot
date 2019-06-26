/*!
* @file     SignalLoggerPlugin.cpp
* @author   Gabriel Hottiger, Samuel Bachmann
* @date     October 10, 2016
* @brief    Signal Logger Rqt plugin.
*/
// rqt_signal_logger
#include "rqt_signal_logger/SignalLoggerPlugin.hpp"

// yaml-cpp
#include <yaml-cpp/yaml.h>

// ros
#include <ros/package.h>
#include <pluginlib/class_list_macros.h>

// Qt
#include <QStringList>
#include <QGridLayout>
#include <QScrollArea>
#include <QScrollBar>
#include <QMessageBox>
#include <QFileDialog>
#include <QSignalMapper>
#include <QCompleter>

// STL
#include <fstream>

// System
#include <sys/stat.h>

namespace rqt_signal_logger {

static bool compareNoCase( const std::string& s1, const std::string& s2 ) {
  return strcasecmp( s1.c_str(), s2.c_str() ) <= 0;
}

struct size_less {
  template<class T> bool operator()(T const &a, T const &b) const
  { return a.size() < b.size(); }
};

static size_t getMaxParamNameWidth(std::vector<std::string> const &lines) {
  //use QFontMetrics this way;
  QFont font("", 0);
  QFontMetrics fm(font);

  if(lines.size() != 0) {
      auto it = std::max_element(lines.begin(), lines.end(), size_less());
      QString text = QString::fromStdString(*it);
      return fm.width(text);
  }

  return 0;
}

SignalLoggerPlugin::SignalLoggerPlugin() :
                            rqt_gui_cpp::Plugin(),
                            widget_(0),
                            tabWidget_(),
                            statusBar_(),
                            varsWidget_(0),
                            configureWidget_(0),
                            paramsGrid_(),
                            paramsWidget_(0),
                            paramsScrollHelperWidget_(0),
                            paramsScrollLayout_(0),
                            updateFrequency_(0.0) {
  setObjectName("SignalLoggerPlugin");
}

void SignalLoggerPlugin::initPlugin(qt_gui_cpp::PluginContext& context) {
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  if (context.serialNumber() > 1) {
    widget_->setWindowTitle(widget_->windowTitle() +
        " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // set black background
  statusBar_ = new QStatusBar(widget_);
  QPalette pal = statusBar_->palette();
  pal.setColor(QPalette::Background, Qt::white);
  statusBar_->setAutoFillBackground(true);
  statusBar_->setSizeGripEnabled(false);
  statusBar_->setPalette(pal);
  statusBar_->show();

  ui_.statusBarLayout->addWidget(statusBar_);

  // create the vars widget add it as first tab
  varsWidget_ = new QWidget(tabWidget_);
  varsUi_.setupUi(varsWidget_);

  // create the configure widget add it as second tab
  configureWidget_ = new QWidget(tabWidget_);
  configureUi_.setupUi(configureWidget_);

  ui_.tabVariables->layout()->addWidget(varsWidget_);
  ui_.tabConfigure->layout()->addWidget(configureWidget_);

  // Do some configuration
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::ENABLE_ALL), "Enable all elements");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::DISABLE_ALL), "Disable all elements");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_DIVIDER), "Set divider to ");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_ACTION), "Set action to ");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_BUFFER_TYPE), "Set buffer type to ");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_BUFFER_SIZE), "Set buffer size to ");
  varsUi_.taskComboBox->insertItem(static_cast<int>(TaskList::SET_BUFFER_SIZE_FROM_TIME), "Setup buffer size from time");
  varsUi_.taskComboBox->setCurrentIndex(static_cast<int>(TaskList::ENABLE_ALL));
  taskChanged(static_cast<int>(TaskList::ENABLE_ALL));
  //varsUi_.valueSpinBox->setMinimumWidth(varsUi_.valueSpinBox->fontMetrics().width(QString("(1) Save and Publish"))+30);

  /******************************
   * Connect ui forms to actions *
   ******************************/
  connect(varsUi_.pushButtonRefreshAll, SIGNAL(pressed()), this, SLOT(refreshAll()));
  connect(varsUi_.lineEditFilter, SIGNAL(returnPressed()), this ,SLOT(refreshAll()));
  connect(varsUi_.pushButtonChangeAll, SIGNAL(pressed()), this, SLOT(changeAll()));
  connect(this, SIGNAL(parametersChanged()), this, SLOT(drawParamList()));
  connect(varsUi_.taskComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(taskChanged(int)));
  connect(varsUi_.applyButton, SIGNAL(pressed()), this, SLOT(applyButtonPressed()));
  connect(varsUi_.valueSpinBox, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));

  connect(configureUi_.startLoggerButton, SIGNAL(pressed()), this, SLOT(startLogger()));
  connect(configureUi_.stopLoggerButton, SIGNAL(pressed()), this, SLOT(stopLogger()));
  connect(configureUi_.restartLoggerButton, SIGNAL(pressed()), this, SLOT(restartLogger()));
  connect(configureUi_.pathButton, SIGNAL(pressed()), this, SLOT(selectYamlFile()));
  connect(configureUi_.loadScriptButton, SIGNAL(pressed()), this, SLOT(loadYamlFile()));
  connect(configureUi_.saveScriptButton, SIGNAL(pressed()), this, SLOT(saveYamlFile()));
  connect(configureUi_.namespaceEdit, SIGNAL(returnPressed()), this, SLOT(setNamespace()));
  connect(configureUi_.saveButton, SIGNAL(pressed()), this, SLOT(saveLoggerData()));

  // Those are fixed in signal_logger_ros
  getLoggerConfigurationServiceName_ = "/silo_ros/get_logger_configuration";
  getParameterServiceName_  = "/silo_ros/get_logger_element";
  setParameterServiceName_ = "/silo_ros/set_logger_element";
  startLoggerServiceName_  = "/silo_ros/start_logger";
  stopLoggerServiceName_ = "/silo_ros/stop_logger";
  saveLoggerDataServiceName_  = "/silo_ros/save_logger_data";
  loadLoggerScriptServiceName_  = "/silo_ros/load_logger_script";
  saveLoggerScriptServiceName_  = "/silo_ros/save_logger_script";
  isLoggerRunningServiceName_ = "/silo_ros/is_logger_running";

  //! Get namespace from rosparam server and add it to the parameter list
  std::string namespaceParameterName = "/user_interface/rqt_signal_logger/logger_namespace";
  if(getNodeHandle().hasParam(namespaceParameterName)) {
    std::string loggerNamespace;
    getNodeHandle().getParam(namespaceParameterName, loggerNamespace);
    if(checkNamespace(QString::fromStdString(loggerNamespace))) {
      configureUi_.namespaceEdit->setText(QString::fromStdString(loggerNamespace));
    }
    else {
      ROS_WARN_STREAM("Invalid logger namespace. Set last used namespace.");
    }
  }
  else {
    ROS_WARN_STREAM("Ros parameter: " << namespaceParameterName << " not found. Set last used namespace.");
  }
}

bool SignalLoggerPlugin::checkNamespace(const QString & text) {
  try {
    return ( ros::service::exists(text.toStdString()+getLoggerConfigurationServiceName_, false) &&
             ros::service::exists(text.toStdString()+getParameterServiceName_, false) &&
             ros::service::exists(text.toStdString()+setParameterServiceName_, false) &&
             ros::service::exists(text.toStdString()+startLoggerServiceName_, false) &&
             ros::service::exists(text.toStdString()+stopLoggerServiceName_, false) &&
             ros::service::exists(text.toStdString()+saveLoggerDataServiceName_, false) &&
             ros::service::exists(text.toStdString()+loadLoggerScriptServiceName_, false) &&
             ros::service::exists(text.toStdString()+saveLoggerScriptServiceName_, false) &&
             ros::service::exists(text.toStdString()+isLoggerRunningServiceName_, false) );
  }
  catch(...)
  {
    return false;
  }
}

void SignalLoggerPlugin::setNamespace() {
  shutdownROS();
  QString text = configureUi_.namespaceEdit->displayText();

  if(checkNamespace(text))
  {
    if( !namespaceList_.contains(text, Qt::CaseSensitive) ) {
      namespaceList_.append(text);
    }
    // ROS services
    getLoggerConfigurationClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerConfiguration>(text.toStdString()+getLoggerConfigurationServiceName_);
    getLoggerElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::GetLoggerElement>(text.toStdString()+getParameterServiceName_);
    setLoggerElementClient_ = getNodeHandle().serviceClient<signal_logger_msgs::SetLoggerElement>(text.toStdString()+setParameterServiceName_);
    startLoggerClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(text.toStdString()+startLoggerServiceName_);
    stopLoggerClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(text.toStdString()+stopLoggerServiceName_);
    saveLoggerDataClient_ = getNodeHandle().serviceClient<signal_logger_msgs::SaveLoggerData>(text.toStdString()+saveLoggerDataServiceName_);
    loadLoggerScriptClient_ = getNodeHandle().serviceClient<signal_logger_msgs::EditLoggerScript>(text.toStdString()+loadLoggerScriptServiceName_);
    saveLoggerScriptClient_ = getNodeHandle().serviceClient<signal_logger_msgs::EditLoggerScript>(text.toStdString()+saveLoggerScriptServiceName_);
    isLoggerRunningClient_ = getNodeHandle().serviceClient<std_srvs::Trigger>(text.toStdString()+isLoggerRunningServiceName_);
    statusMessage(std::string("Found services for namespace ")+ text.toStdString() + std::string("! Refresh log elements!") , MessageType::SUCCESS);
    refreshAll();
  }
  else {
    statusMessage(std::string("Could not find services for namespace ")+ text.toStdString() + std::string("!") , MessageType::ERROR);
  }

  QCompleter *completer = new QCompleter(namespaceList_, this);
  completer->setCaseSensitivity(Qt::CaseSensitive);
  configureUi_.namespaceEdit->setCompleter(completer);
}

void SignalLoggerPlugin::shutdownROS() {
  getLoggerConfigurationClient_.shutdown();
  getLoggerElementClient_.shutdown();
  setLoggerElementClient_.shutdown();
  startLoggerClient_.shutdown();
  stopLoggerClient_.shutdown();
  saveLoggerDataClient_.shutdown();
  loadLoggerScriptClient_.shutdown();
  saveLoggerScriptClient_.shutdown();
  isLoggerRunningClient_.shutdown();
}

void SignalLoggerPlugin::changeAll() {
  bool success = true;

  for (auto& elem : logElements_) {
    success = success && elem->changeElement();
  }
  if(success) {
    statusMessage(std::string("Applied changes to all the logger elements!"), MessageType::STATUS);
  }
  else {
    statusMessage(std::string("Could not apply changes to all the logger elements!"), MessageType::WARNING);
  }
}

void SignalLoggerPlugin::refreshAll() {
  signal_logger_msgs::GetLoggerConfigurationRequest req;
  signal_logger_msgs::GetLoggerConfigurationResponse res;

  if (getLoggerConfigurationClient_.call(req,res)) {
    // Update parameter names
    logElementNames_ = res.log_element_names;
    updateFrequency_ = res.collect_frequency;
    configureUi_.pathEdit->setText(QString::fromStdString(res.script_filepath));
    varsUi_.ns->setText(QString::fromStdString(res.logger_namespace));
    varsUi_.freq->setText(QString::number(updateFrequency_, 'f', 2) +
                              QString::fromStdString(" [Hz]"));

    // Sort names alphabetically.
    std::sort(logElementNames_.begin(), logElementNames_.end(), compareNoCase );

    // Update GUI
    emit parametersChanged();
  }
  else {
    statusMessage(std::string("Could not get current logger configuration."), MessageType::WARNING);
  }
}

void SignalLoggerPlugin::startLogger() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  if(startLoggerClient_.call(req, res) && res.success) {
    statusMessage("Successfully started logger!", MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not start logger! Already running?", MessageType::WARNING, 2.0);
  }
  checkLoggerState();
}

void SignalLoggerPlugin::stopLogger() {
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  if(stopLoggerClient_.call(req, res) && res.success) {
    statusMessage("Successfully stopped logger!", MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not stop logger! Not running?", MessageType::WARNING, 2.0);
  }
  checkLoggerState();
}

void SignalLoggerPlugin::restartLogger() {
  stopLogger();
  startLogger();
}

void SignalLoggerPlugin::saveLoggerData() {
  signal_logger_msgs::SaveLoggerData::Request req;
  signal_logger_msgs::SaveLoggerData::Response res;
  if(configureUi_.binaryButton->isChecked()) { req.logfileTypes.push_back(signal_logger_msgs::SaveLoggerData::Request::LOGFILE_TYPE_BINARY); }
  if(configureUi_.csvButton->isChecked()) { req.logfileTypes.push_back(signal_logger_msgs::SaveLoggerData::Request::LOGFILE_TYPE_CSV); }
  if(configureUi_.bagButton->isChecked()) { req.logfileTypes.push_back(signal_logger_msgs::SaveLoggerData::Request::LOGFILE_TYPE_BAG); }

  if(saveLoggerDataClient_.call(req, res) && res.success) {
    std::string msg = std::string{"Successfully saved logger data to file."};
    statusMessage(msg, MessageType::SUCCESS, 2.0);
  }
  else {
    statusMessage("Could not save logger data!", MessageType::WARNING, 2.0);
  }
}

void SignalLoggerPlugin::selectYamlFile() {
  QString fileName = QFileDialog::getOpenFileName(configureWidget_,
                                                  tr("Open Logger Configuration File"), "/home", tr("YAML Files (*.yaml)"));
  configureUi_.pathEdit->setText(fileName);
}

void SignalLoggerPlugin::loadYamlFile() {
  signal_logger_msgs::EditLoggerScriptRequest req;
  signal_logger_msgs::EditLoggerScriptResponse res;
  req.filepath = configureUi_.pathEdit->text().toStdString();
  if(loadLoggerScriptClient_.call(req, res) && res.success) {
    statusMessage(std::string("Successfully loaded logger configuration file: ") + req.filepath, MessageType::SUCCESS, 2.0);
    refreshAll();
  }
  else {
    statusMessage(std::string("Could not load logger configuration file: ") + req.filepath + std::string("! Is logger running?"), MessageType::WARNING, 2.0);
  }
}

void SignalLoggerPlugin::saveYamlFile() {
  // Check for file existance and ask user for overwrite
/*  std::string filename = configureUi_.pathEdit->text().toStdString();
  struct stat buffer;
  if(stat(filename.c_str(), &buffer) == 0)
  {
    QMessageBox msgBox;
    msgBox.setText("The configuration file already exists.");
    msgBox.setInformativeText("Do you want to overwrite it?");
    msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    msgBox.setDefaultButton(QMessageBox::Cancel);
    if(msgBox.exec() == QMessageBox::Cancel)
    {
      statusMessage("Did not save logger configuration file!", MessageType::WARNING, 2.0);
      return;
    }
  }*/

  signal_logger_msgs::EditLoggerScriptRequest req;
  signal_logger_msgs::EditLoggerScriptResponse res;
  req.filepath = configureUi_.pathEdit->text().toStdString();
  if(saveLoggerScriptClient_.call(req, res) && res.success) {
    statusMessage(std::string("Successfully saved logger configuration file: ") + req.filepath, MessageType::SUCCESS, 2.0);
    refreshAll();
  }
  else {
    statusMessage(std::string("Could not save logger configuration file: ") + req.filepath + std::string("! Is logger running?"), MessageType::WARNING, 2.0);
  }

}

void SignalLoggerPlugin::taskChanged(int index) {
  // Restore Default
  varsUi_.valueSpinBox->setEnabled(true);
  varsUi_.valueSpinBox->setSuffix(QString::fromUtf8(""));
  varsUi_.valueSpinBox->setPrefix(QString::fromUtf8(""));
  varsUi_.valueSpinBox->setValue(0);
  varsUi_.valueSpinBox->setMinimum(0);
  varsUi_.valueSpinBox->setSingleStep(1);
  varsUi_.valueSpinBox->setDecimals(0);

  // Change depending on type
  switch(index) {
    case static_cast<int>(TaskList::ENABLE_ALL):
    case static_cast<int>(TaskList::DISABLE_ALL):
      varsUi_.valueSpinBox->setEnabled(false);
      break;
    case static_cast<int>(TaskList::SET_BUFFER_TYPE):
    case static_cast<int>(TaskList::SET_ACTION):
      varsUi_.valueSpinBox->setPrefix(QString::fromUtf8("("));
      valueChanged(0);
      break;
    case static_cast<int>(TaskList::SET_BUFFER_SIZE):
    case static_cast<int>(TaskList::SET_DIVIDER):
      varsUi_.valueSpinBox->setRange(0.0, 100000.0);
      break;
    case static_cast<int>(TaskList::SET_BUFFER_SIZE_FROM_TIME):
      varsUi_.valueSpinBox->setSuffix(QString::fromUtf8(" [sec]"));
      varsUi_.valueSpinBox->setDecimals(2);
      varsUi_.valueSpinBox->setRange(0.0, 600.0);
      break;
    default:
      break;
  }
}

void SignalLoggerPlugin::valueChanged(double value) {
  switch(varsUi_.taskComboBox->currentIndex()) {
     case static_cast<int>(TaskList::SET_ACTION):
       {
         varsUi_.valueSpinBox->setValue(((int)roundf(value))%3);
         switch(((int)roundf(value))%3) {
           case static_cast<int>(LogElement::LogAction::SAVE_AND_PUBLISH):
              varsUi_.valueSpinBox->setSuffix(") Save and Publish");
              break;
           case static_cast<int>(LogElement::LogAction::SAVE):
              varsUi_.valueSpinBox->setSuffix(") Save");
              break;
           case static_cast<int>(LogElement::LogAction::PUBLISH):
              varsUi_.valueSpinBox->setSuffix(") Publish");
              break;
           default:
              break;
         }
       }
       break;
     case static_cast<int>(TaskList::SET_BUFFER_TYPE):
       {
         varsUi_.valueSpinBox->setValue(((int)roundf(value))%3);
         switch(((int)roundf(value))%3) {
           case static_cast<int>(LogElement::BufferType::FIXED_SIZE):
              varsUi_.valueSpinBox->setSuffix(") Fixed Size");
              break;
           case static_cast<int>(LogElement::BufferType::LOOPING):
              varsUi_.valueSpinBox->setSuffix(") Looping");
              break;
           case static_cast<int>(LogElement::BufferType::EXPONENTIALLY_GROWING):
              varsUi_.valueSpinBox->setSuffix(") Growing");
              break;
           default:
              break;
         }
       }
       break;
     default:
       break;
   }
}

void SignalLoggerPlugin::applyButtonPressed() {

  switch(varsUi_.taskComboBox->currentIndex()) {
    case static_cast<int>(TaskList::ENABLE_ALL):
    {
      for(auto & element : logElements_) {
        element->checkBoxIsLogging->setCheckState(Qt::CheckState::Checked);
      }
      statusMessage("Enabled all elements. Press 'Change All' to apply.", MessageType::SUCCESS, 2.0);
      break;
    }
    case static_cast<int>(TaskList::DISABLE_ALL):
    {
      for(auto & element : logElements_) {
        element->checkBoxIsLogging->setCheckState(Qt::CheckState::Unchecked);
      }
      statusMessage("Disabled all elements. Press 'Change All' to apply.", MessageType::SUCCESS, 2.0);
      break;
    }
    case static_cast<int>(TaskList::SET_DIVIDER):
    {
      for(auto & element : logElements_) {
        element->spinBoxDivider->setValue(varsUi_.valueSpinBox->value());
      }
      statusMessage(std::string("Set all dividers to ") + std::to_string(varsUi_.valueSpinBox->value())
                    + std::string(". Press 'Change All' to apply.") , MessageType::SUCCESS, 2.0);
      break;
    }
    case static_cast<int>(TaskList::SET_ACTION):
    {
      for(auto & element : logElements_) {
        element->comboBoxLogType->setCurrentIndex((int)varsUi_.valueSpinBox->value());
      }
      std::string action = varsUi_.valueSpinBox->suffix().toStdString();
      action = action.substr(2, action.size() - 2);
      statusMessage(std::string("Set all dividers to ") + action + std::string(". Press 'Change All' to apply.") , MessageType::SUCCESS, 2.0);
      break;
    }
    case static_cast<int>(TaskList::SET_BUFFER_TYPE):
    {
      for(auto & element : logElements_) {
        element->comboBoxBufferType->setCurrentIndex((int)varsUi_.valueSpinBox->value());
      }
      std::string type = varsUi_.valueSpinBox->suffix().toStdString();
      type = type.substr(2, type.size() - 2);
      statusMessage(std::string("Set all buffer types to ") + type + std::string(". Press 'Change All' to apply.") , MessageType::SUCCESS, 2.0);
      break;
    }
    case static_cast<int>(TaskList::SET_BUFFER_SIZE):
    {
      for(auto & element : logElements_) {
        element->spinBoxBufferSize->setValue(varsUi_.valueSpinBox->value());
      }
      statusMessage(std::string("Set all buffer sizes to ") + std::to_string(varsUi_.valueSpinBox->value())
                  + std::string(". Press 'Change All' to apply.") , MessageType::SUCCESS, 2.0);
      break;
    }
    case static_cast<int>(TaskList::SET_BUFFER_SIZE_FROM_TIME):
    {
      for(auto & element : logElements_) {
        element->spinBoxBufferSize->setValue(ceil((double) varsUi_.valueSpinBox->value() * updateFrequency_ / (double)element->spinBoxDivider->value()));
      }
      statusMessage(std::string("Set all buffer sizes from time ") + std::to_string(varsUi_.valueSpinBox->value())
                  + std::string(". Press 'Change All' to apply.") , MessageType::SUCCESS, 2.0);
      break;
    }
    default:
      break;
  }

}

void SignalLoggerPlugin::checkLoggerState() {
  std_srvs::TriggerRequest req_islogging;
  std_srvs::TriggerResponse res_islogging;

  if (isLoggerRunningClient_.call(req_islogging, res_islogging)) {
    varsUi_.pushButtonChangeAll->setEnabled(!res_islogging.success);
    for(auto & element  : logElements_) {
      element->pushButtonChangeParam->setEnabled(!res_islogging.success);
    }
  }
  else {
    statusMessage("Can not check if logger is running.", MessageType::WARNING, 2.0);
  }

  return;
}

void SignalLoggerPlugin::shutdownPlugin() {
  shutdownROS();
}

void SignalLoggerPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {
  plugin_settings.setValue("namespaceEdit", configureUi_.namespaceEdit->displayText());
  plugin_settings.setValue("namespaceList", namespaceList_);
  plugin_settings.setValue("lineEditFilter", varsUi_.lineEditFilter->displayText());
  plugin_settings.setValue("pathEdit", configureUi_.pathEdit->displayText());
}

void SignalLoggerPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {
  varsUi_.lineEditFilter->setText(plugin_settings.value("lineEditFilter").toString());
  configureUi_.pathEdit->setText(plugin_settings.value("pathEdit").toString());
  namespaceList_ = plugin_settings.value("namespaceList").toStringList();

  if(configureUi_.namespaceEdit->displayText().isEmpty()) {
    configureUi_.namespaceEdit->setText(plugin_settings.value("namespaceEdit").toString());
  }

  setNamespace();
}

// Try to find the Needle in the Haystack - ignore case
// Taken from http://stackoverflow.com/questions/3152241/case-insensitive-stdstring-find
bool findStringIC(const std::string & strHaystack, const std::string & strNeedle)
{
  auto it = std::search(
      strHaystack.begin(), strHaystack.end(),
      strNeedle.begin(),   strNeedle.end(),
      [](char ch1, char ch2) { return std::toupper(ch1) == std::toupper(ch2); }
  );
  return (it != strHaystack.end() );
}

void SignalLoggerPlugin::drawParamList() {

  logElements_.clear();

  if (paramsWidget_) {
    // delete widget
    delete paramsWidget_->layout();
    delete paramsScrollHelperWidget_->layout();
    varsUi_.verticalLayout->removeWidget(paramsWidget_);
    delete paramsWidget_;
  }
  varsUi_.verticalSpacer->changeSize(1, 1, QSizePolicy::Fixed,
                                     QSizePolicy::Fixed);

  paramsWidget_ = new QWidget();
  paramsWidget_->setObjectName(QString::fromUtf8("paramsWidget"));
  varsUi_.verticalLayout->insertWidget(2, paramsWidget_);

  paramsScrollHelperWidget_ = new QWidget(paramsWidget_);
  paramsGrid_= new QGridLayout(paramsScrollHelperWidget_);
  paramsGrid_->setObjectName(QString::fromUtf8("paramsGrid"));

  const size_t maxParamNameWidth = getMaxParamNameWidth(logElementNames_);

  // Create a line for each filtered parameter
  std::string filter = varsUi_.lineEditFilter->text().toStdString();
  for (auto& name : logElementNames_) {
    //    std::size_t found = name.find(filter);
    if (findStringIC(name, filter)) {
      logElements_.push_back(std::shared_ptr<LogElement>(new LogElement(name, varsWidget_, paramsGrid_, &getLoggerElementClient_, &setLoggerElementClient_, maxParamNameWidth)));
    }
  }
  // This needs to be done after everthing is setup.
  paramsScrollHelperWidget_->setLayout(paramsGrid_);

  // Put it into a scroll area
  QScrollArea* paramsScrollArea = new QScrollArea();
  paramsScrollArea->setWidget(paramsScrollHelperWidget_);

  // Make the scroll step the same width as the fixed widgets in the grid
  paramsScrollArea->horizontalScrollBar()->setSingleStep(paramsScrollHelperWidget_->width() / 24);

  paramsScrollLayout_ = new QVBoxLayout(paramsWidget_);
  paramsScrollLayout_->addWidget(paramsScrollArea);

  paramsWidget_->setLayout(paramsScrollLayout_);
  paramsScrollHelperWidget_->setLayout(paramsGrid_);

  this->checkLoggerState();

}

void SignalLoggerPlugin::statusMessage(std::string message, MessageType type, double displaySeconds) {
  switch(type) {
    case MessageType::ERROR:
      statusBar_->setStyleSheet("color: red");
      ROS_ERROR_STREAM(message.c_str());
      break;
    case MessageType::WARNING:
      statusBar_->setStyleSheet("color: orange");
      ROS_WARN_STREAM(message.c_str());
      break;
    case MessageType::SUCCESS:
      statusBar_->setStyleSheet("color: green");
      ROS_INFO_STREAM(message.c_str());
      break;
    case MessageType::STATUS:
      statusBar_->setStyleSheet("color: black");
      ROS_INFO_STREAM(message.c_str());
      break;
  }
  statusBar_->showMessage(QString::fromStdString(message), displaySeconds*1000);
}

}

PLUGINLIB_EXPORT_CLASS(rqt_signal_logger::SignalLoggerPlugin, rqt_gui_cpp::Plugin)
