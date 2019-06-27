#include <rocoma_ros/ControllerManagerRos.hpp>
#include <rocoma/ControllerManager.hpp>
#include <ros/package.h>
#include <future>

namespace rocoma_ros {

template<typename State_, typename Command_>
ControllerManagerRos<State_,Command_>::ControllerManagerRos( const std::string & scopedStateName,
                                                             const std::string & scopedCommandName):
                                                             rocoma::ControllerManager(),
                                                             isInitializedRos_(false),
                                                             nodeHandle_(),
                                                             switchControllerService_(),
                                                             emergencyStopService_(),
                                                             failproofStopService_(),
                                                             clearEmergencyStopService_(),
                                                             getAvailableControllersService_(),
                                                             getActiveControllerService_(),
                                                             activeControllerPublisher_(),
                                                             activeControllerMsg_(),
                                                             controllerManagerStatePublisher_(),
                                                             controllerManagerStateMsg_(),
                                                             emergencyStopStatePublisher_(),
                                                             emergencyStopStateMsg_(),
                                                             failproofControllerLoader_("rocoma_plugin", "rocoma_plugin::FailproofControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                             emergencyControllerLoader_("rocoma_plugin", "rocoma_plugin::EmergencyControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                             emergencyControllerRosLoader_("rocoma_plugin", "rocoma_plugin::EmergencyControllerRosPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                             controllerLoader_("rocoma_plugin", "rocoma_plugin::ControllerPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                             controllerRosLoader_("rocoma_plugin", "rocoma_plugin::ControllerRosPluginInterface<" + scopedStateName + ", " + scopedCommandName + ">"),
                                                             sharedModuleLoader_("rocoma_plugin", "rocoma_plugin::SharedModulePluginInterface"),
                                                             sharedModuleRosLoader_("rocoma_plugin", "rocoma_plugin::SharedModuleRosPluginInterface")
{

}

template<typename State_, typename Command_>
ControllerManagerRos<State_,Command_>::ControllerManagerRos( const std::string & scopedStateName,
                                                             const std::string & scopedCommandName,
                                                             const ControllerManagerRosOptions & options):
      ControllerManagerRos()
{
  this->init(options);
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::init(const ControllerManagerRosOptions & options) {
  if(isInitializedRos_) {
    MELO_WARN("[RocomaRos] Already initialized. Kept current configuration.");
    return;
  }

  // Init controller manager
  rocoma::ControllerManager::init(options);
  nodeHandle_ = options.nodeHandle;

  // Shutdown publishers
  shutdown();

  // initialize services
  std::string service_name_switch_controller{"controller_manager/switch_controller"};
  nodeHandle_.getParam("servers/switch_controller/service", service_name_switch_controller);
  switchControllerService_ = nodeHandle_.advertiseService(service_name_switch_controller, &ControllerManagerRos::switchControllerService, this);

  std::string service_name_get_available_controllers{"controller_manager/get_available_controllers"};
  nodeHandle_.getParam("servers/get_available_controllers/service", service_name_get_available_controllers);
  getAvailableControllersService_ = nodeHandle_.advertiseService(service_name_get_available_controllers, &ControllerManagerRos::getAvailableControllersService, this);

  std::string service_name_get_active_controller{"controller_manager/get_active_controller"};
  nodeHandle_.getParam("servers/get_active_controller/service", service_name_get_active_controller);
  getActiveControllerService_ = nodeHandle_.advertiseService(service_name_get_active_controller, &ControllerManagerRos::getActiveControllerService, this);

  std::string service_name_emergency_stop{"controller_manager/emergency_stop"};
  nodeHandle_.getParam("servers/emergency_stop/service", service_name_emergency_stop);
  emergencyStopService_ = nodeHandle_.advertiseService(service_name_emergency_stop, &ControllerManagerRos::emergencyStopService, this);

  std::string service_name_failproof_stop{"controller_manager/failproof_stop"};
  nodeHandle_.getParam("servers/failproof_stop/service", service_name_failproof_stop);
  failproofStopService_ = nodeHandle_.advertiseService(service_name_failproof_stop, &ControllerManagerRos::failproofStopService, this);

  std::string service_name_clear_emergency_stop{"controller_manager/clear_emergency_stop"};
  nodeHandle_.getParam("servers/clear_emergency_stop/service", service_name_clear_emergency_stop);
  clearEmergencyStopService_ = nodeHandle_.advertiseService(service_name_clear_emergency_stop, &ControllerManagerRos::clearEmergencyStopService, this);

  // initialize publishers
  std::string topic_name_notify_active_controller{"notify_active_controller"};
  nodeHandle_.getParam("publishers/notify_active_controller/topic", topic_name_notify_active_controller);
  activeControllerPublisher_ = nodeHandle_.advertise<std_msgs::String>(topic_name_notify_active_controller, 1, true);
  publishActiveController(this->getActiveControllerName());

  std::string topic_name_notify_controller_manager_state{"notify_controller_manager_state"};
  nodeHandle_.getParam("publishers/notify_controller_manager_state/topic", topic_name_notify_controller_manager_state);
  controllerManagerStatePublisher_ = nodeHandle_.advertise<rocoma_msgs::ControllerManagerState>(topic_name_notify_controller_manager_state, 1, true);
  publishControllerManagerState(this->getControllerManagerState(), this->hasClearedEmergencyStop());

  std::string topic_name_notify_emergency_stop{"notify_emergency_stop"};
  nodeHandle_.getParam("publishers/notify_emergency_stop/topic", topic_name_notify_emergency_stop);
  emergencyStopStatePublisher_ = nodeHandle_.advertise<rocoma_msgs::EmergencyStop>(topic_name_notify_emergency_stop, 1, true);
  publishEmergencyState(false);

  // Set init flag
  isInitializedRos_ = true;
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::shutdown() {
  switchControllerService_.shutdown();
  getAvailableControllersService_.shutdown();
  getActiveControllerService_.shutdown();
  emergencyStopService_.shutdown();
  failproofStopService_.shutdown();
  clearEmergencyStopService_.shutdown();
  controllerManagerStatePublisher_.shutdown();
  emergencyStopStatePublisher_.shutdown();
  activeControllerPublisher_.shutdown();
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::cleanup() {
  bool success = rocoma::ControllerManager::cleanup();
  shutdown();
  return success;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupControllerPair(const ManagedControllerOptionsPair & options,
                                                                std::shared_ptr<State_> state,
                                                                std::shared_ptr<Command_> command,
                                                                std::shared_ptr<boost::shared_mutex> mutexState,
                                                                std::shared_ptr<boost::shared_mutex> mutexCommand) {
  if(!isInitializedRos_) {
    MELO_ERROR("[RocomaRos] Not initialized. Can not setup controller.");
    return false;
  }
  //--- Add controller
  rocoma_plugin::ControllerPluginInterface<State_, Command_> * controller;

  try
  {
    if(options.first.isRos_) {
      // Instantiate controller
      rocoma_plugin::ControllerRosPluginInterface<State_, Command_> * rosController = controllerRosLoader_.createUnmanagedInstance(options.first.pluginName_);
      // Set node handle
      rosController->setNodeHandle(nodeHandle_);
      controller = rosController;
    }
    else {
      controller = controllerLoader_.createUnmanagedInstance(options.first.pluginName_);
    }

    // Set state and command
    controller->setName( options.first.name_ );
    controller->setStateAndCommand(state, mutexState, command, mutexCommand);
    controller->setParameterPath(options.first.parameterPath_);
    for(auto & sharedModuleName : options.first.sharedModuleNames_) {
      if(this->hasSharedModule(sharedModuleName)) {
        controller->addSharedModule( sharedModules_.at(sharedModuleName) );
      } else {
        MELO_WARN("[RocomaRos] Shared module %s does not exist. Failed to add it to the controller %s.", sharedModuleName.c_str(), controller->getName().c_str());
      }
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    //handle the class failing to load
    MELO_ERROR("[RocomaRos] The plugin failed to load for some reason. Error: %s", ex.what());
    MELO_WARN_STREAM("[RocomaRos] Could not setup controller: " << options.first.name_ << "!");
    return false;
  }


  //--- Add emergency controller
  rocoma_plugin::EmergencyControllerPluginInterface<State_, Command_> * emgcyController = nullptr;

  if(!options.second.name_.empty())
  {
    // Load shared modules
    for(auto & sharedModuleName : options.second.sharedModuleNames_) {
      if(!this->hasSharedModule(sharedModuleName)) {
        this->addSharedModule( roco::SharedModulePtr( sharedModuleLoader_.createUnmanagedInstance(sharedModuleName) ) );
      }
      MELO_INFO_STREAM("[RocomaRos] Added shared module " << sharedModuleName << " to emergency controller " << options.second.name_ << "!");
    }

    try
    {
      if(options.second.isRos_) {
        // Instantiate controller
        rocoma_plugin::EmergencyControllerRosPluginInterface<State_, Command_> * rosEmergencyController =
            emergencyControllerRosLoader_.createUnmanagedInstance(options.second.pluginName_);
        // Set node handle
        rosEmergencyController->setNodeHandle(nodeHandle_);
        emgcyController = rosEmergencyController;
      }
      else {
        emgcyController = emergencyControllerLoader_.createUnmanagedInstance(options.second.pluginName_);
      }

      // Set state and command
      emgcyController->setName( options.second.name_ );
      emgcyController->setStateAndCommand(state, mutexState, command, mutexCommand);
      emgcyController->setParameterPath(options.second.parameterPath_);
      for(auto & sharedModuleName : options.second.sharedModuleNames_) {
        if(this->hasSharedModule(sharedModuleName)) {
          emgcyController->addSharedModule( sharedModules_.at(sharedModuleName) );
        } else {
          MELO_WARN("[RocomaRos] Shared module %s does not exist. Failed to add it to the emergency controller %s.", sharedModuleName.c_str(), emgcyController->getName().c_str());
        }
      }

    }
    catch(pluginlib::PluginlibException& ex)
    {
      //handle the class failing to load
      MELO_WARN("[RocomaRos] The plugin failed to load for some reason. Error: %s", ex.what());
      MELO_WARN_STREAM("[RocomaRos] Could not setup emergency controller: " << options.second.name_ << "! Using failproof controller instead");
      emgcyController = nullptr;
    }
  } // endif

  // Set name to failproof if nullptr
  std::string emergencyControllerName = (emgcyController == nullptr)? "FailproofController" : options.second.name_;

  // Add controller to the manager
  if(!this->addControllerPair(std::unique_ptr< roco::ControllerAdapterInterface >(controller),
                              std::unique_ptr< roco::EmergencyControllerAdapterInterface >(emgcyController)))
  {
    MELO_WARN_STREAM("[RocomaRos] Could not add controller pair ( " << options.first.name_ << " / "
                     << emergencyControllerName << " ) to controller manager!");
    return false;
  }

  // Inform user
  MELO_INFO_STREAM("[RocomaRos] Successfully added controller pair ( " << options.first.name_ << " / "
                   << emergencyControllerName << " ) to controller manager!");

  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupFailproofController(const std::string & controllerPluginName,
                                                                     std::shared_ptr<State_> state,
                                                                     std::shared_ptr<Command_> command,
                                                                     std::shared_ptr<boost::shared_mutex> mutexState,
                                                                     std::shared_ptr<boost::shared_mutex> mutexCommand) {
  if(!isInitializedRos_) {
    MELO_ERROR("[RocomaRos] Not initialized. Can not setup failproof controller.");
    return false;
  }
  try
  {
    // Instantiate controller
    rocoma_plugin::FailproofControllerPluginInterface<State_, Command_> * controller =
        failproofControllerLoader_.createUnmanagedInstance(controllerPluginName);

    // Set state and command
    controller->setStateAndCommand(state, mutexState, command, mutexCommand);

    // Add controller to the manager
    if(!this->setFailproofController(std::unique_ptr< rocoma_plugin::FailproofControllerPluginInterface<State_,Command_> >(controller))) {
      MELO_WARN_STREAM("[RocomaRos] Could not add failproof controller: " << controllerPluginName << " to controller manager!");
      return false;
    }

    // Inform user
    MELO_INFO_STREAM("[RocomaRos] Succesfully setup failproof controller: " << controllerPluginName << "!");

  }
  catch(pluginlib::PluginlibException& ex)
  {
    //handle the class failing to load
    MELO_ERROR("[RocomaRos] The plugin failed to load for some reason. Error: %s", ex.what());
    MELO_WARN_STREAM("[RocomaRos] Could not setup failproof controller: " << controllerPluginName << "!");
    return false;
  }

  return true;

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupControllers(const std::string & failproofControllerName,
                                                             const std::vector< ManagedControllerOptionsPair > & controllerNameMap,
                                                             std::shared_ptr<State_> state,
                                                             std::shared_ptr<Command_> command,
                                                             std::shared_ptr<boost::shared_mutex> mutexState,
                                                             std::shared_ptr<boost::shared_mutex> mutexCommand) {
  if(!isInitializedRos_) {
    MELO_ERROR("[RocomaRos] Not initialized. Can not setup controllers.");
    return false;
  }

  // add failproof controller to manager
  bool success = setupFailproofController(failproofControllerName, state, command, mutexState, mutexCommand);

  // add emergency controllers to manager
  for(auto& controllerPair : controllerNameMap)
  {
    success = setupControllerPair(controllerPair, state, command, mutexState, mutexCommand) && success;
  }

  return success;

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupSharedModules(const std::vector<ManagedModuleOptions> & sharedModuleOptions) {
  if(!isInitializedRos_) {
    MELO_ERROR("[RocomaRos] Not initialized. Can not setup shared modules.");
    return false;
  }

  // add emergency controllers to manager
  for(auto& sharedModuleOption : sharedModuleOptions)
  {
    roco::SharedModule* sharedModule;
    if(sharedModuleOption.isRos_) {
      roco_ros::SharedModuleRos* sharedModuleRos = sharedModuleRosLoader_.createUnmanagedInstance(sharedModuleOption.pluginName_);
      sharedModuleRos->setNodeHandle(nodeHandle_);
      sharedModule = sharedModuleRos;
    } else {
      sharedModule = sharedModuleLoader_.createUnmanagedInstance(sharedModuleOption.pluginName_);
    }
    sharedModule->setName(sharedModuleOption.name_);
    sharedModule->setParameterPath(sharedModuleOption.parameterPath_);
    if( sharedModule->create(options_.timeStep) ) {
      this->addSharedModule( roco::SharedModulePtr(sharedModule) );
    }
    else {
      delete sharedModule;
    }
  }

  return true;
}


template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::setupControllersFromParameterServer(std::shared_ptr<State_> state,
                                                                                std::shared_ptr<Command_> command,
                                                                                std::shared_ptr<boost::shared_mutex> mutexState,
                                                                                std::shared_ptr<boost::shared_mutex> mutexCommand) {
  if(!isInitializedRos_) {
    MELO_ERROR("[RocomaRos] Not initialized. Can not setup controllers from parameter server.");
    return false;
  }

  // Parse failproof controller name
  std::string failproofControllerName;
  if(!nodeHandle_.getParam("controller_manager/failproof_controller", failproofControllerName)) {
    MELO_ERROR("[RocomaRos] Could not load parameter 'controller_manager/failproof_controller' from parameter server. Abort.");
    exit(-1);
    return false;
  }

  // Parse shared module list
  std::vector<ManagedModuleOptions> shared_module_option_list;
  ManagedModuleOptions shared_module_option;
  XmlRpc::XmlRpcValue shared_module_list;
  XmlRpc::XmlRpcValue shared_module;

  if(nodeHandle_.getParam("controller_manager/shared_modules", shared_module_list)) {
    if( shared_module_list.getType() == XmlRpc::XmlRpcValue::TypeArray ) {
      for( int i = 0; i < shared_module_list.size(); ++i ) {

        // Check that shared_module exists
        if(shared_module_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
            !shared_module_list[i].hasMember("shared_module") ||
            shared_module_list[i]["shared_module"].getType() != XmlRpc::XmlRpcValue::TypeStruct )
        {
          MELO_WARN("[RocomaRos] Shared module no %d can not be obtained. Skip module.", i);
          continue;
        }

        shared_module = shared_module_list[i]["shared_module"];

        // Check for data members
        if( shared_module.hasMember("plugin_name") &&
            shared_module["plugin_name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            shared_module.hasMember("name") &&
            shared_module["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            shared_module.hasMember("is_ros") &&
            shared_module["is_ros"].getType() == XmlRpc::XmlRpcValue::TypeBoolean &&
            shared_module.hasMember("parameter_package") &&
            shared_module["parameter_package"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            shared_module.hasMember("parameter_path") &&
            shared_module["parameter_path"].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          shared_module_option.pluginName_ = static_cast<std::string>(shared_module["plugin_name"]);
          shared_module_option.name_ = static_cast<std::string>(shared_module["name"]);
          shared_module_option.isRos_ = static_cast<bool>(shared_module["is_ros"]);
          std::string parameterPackage = static_cast<std::string>(shared_module["parameter_package"]);
          if(!parameterPackage.empty()) { parameterPackage = ros::package::getPath(parameterPackage); }
          shared_module_option.parameterPath_ = parameterPackage + "/" + static_cast<std::string>(shared_module["parameter_path"]);
          MELO_INFO("[RocomaRos] Got shared module %s successfully from the parameter server. \n (is_ros: %s, complete parameter_path: %s!)",
                    shared_module_option.pluginName_.c_str(), shared_module_option.isRos_?"true":"false", shared_module_option.parameterPath_.c_str());
          shared_module_option_list.push_back(shared_module_option);
        }
        else
        {
          MELO_WARN("[RocomaRos] Shared module %d has missing or wrong-type entries. Skip module.", i);
          continue;
        }
      }
    }
  }

  // Setup shared modules
  this->setupSharedModules(shared_module_option_list);

  // Parse controller list
  std::vector<ManagedControllerOptionsPair> controller_option_pairs;
  ManagedControllerOptionsPair controller_option_pair;
  XmlRpc::XmlRpcValue controller_pair_list;
  XmlRpc::XmlRpcValue controller;
  if(!nodeHandle_.getParam("controller_manager/controller_pairs", controller_pair_list)) {
    MELO_WARN("[RocomaRos] Could not load parameter 'controller_manager/controller_pairs'. Add only failproof controller.");
  }
  else {
    if(controller_pair_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
    {
      for (int i = 0; i < controller_pair_list.size(); ++i)
      {
        // Check that controller_pair exists
        if(controller_pair_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct ||
           !controller_pair_list[i].hasMember("controller_pair") ||
           controller_pair_list[i]["controller_pair"].getType() != XmlRpc::XmlRpcValue::TypeStruct )
        {
          MELO_WARN("[RocomaRos] Controllerpair no %d can not be obtained. Skip controller pair.", i);
          continue;
        }

        // Check if controller exists
        if(controller_pair_list[i]["controller_pair"].hasMember("controller") &&
            controller_pair_list[i]["controller_pair"]["controller"].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          controller = controller_pair_list[i]["controller_pair"]["controller"];
        }
        else
        {
          MELO_WARN("[RocomaRos] Controllerpair no %d has no or wrong-typed member controller. Skip controller pair.", i);
          continue;
        }

        // Check for data members
        if( controller.hasMember("plugin_name") &&
            controller["plugin_name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            controller.hasMember("name") &&
            controller["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            controller.hasMember("is_ros") &&
            controller["is_ros"].getType() == XmlRpc::XmlRpcValue::TypeBoolean &&
            controller.hasMember("parameter_package") &&
            controller["parameter_package"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            controller.hasMember("parameter_path") &&
            controller["parameter_path"].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          controller_option_pair.first.pluginName_ = static_cast<std::string>(controller["plugin_name"]);
          controller_option_pair.first.name_ = static_cast<std::string>(controller["name"]);
          controller_option_pair.first.isRos_ = static_cast<bool>(controller["is_ros"]);
          std::string parameterPackage = static_cast<std::string>(controller["parameter_package"]);
          if(!parameterPackage.empty()) { parameterPackage = ros::package::getPath(parameterPackage); }
          controller_option_pair.first.parameterPath_ = parameterPackage + "/" + static_cast<std::string>(controller["parameter_path"]);
          XmlRpc::XmlRpcValue shared_modules;
          controller_option_pair.first.sharedModuleNames_.clear();
          if(controller.hasMember("shared_modules") && controller["shared_modules"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int j = 0; j < controller["shared_modules"].size(); ++j) {
              if(controller["shared_modules"][j].getType() == XmlRpc::XmlRpcValue::TypeString) {
                controller_option_pair.first.sharedModuleNames_.push_back(controller["shared_modules"][j]);
              }
            }
          }
          MELO_INFO("[RocomaRos] Got controller plugin %s with controller name %s successfully from the parameter server. \n (is_ros: %s, complete parameter_path: %s!)",
                    controller_option_pair.first.pluginName_.c_str(), controller_option_pair.first.name_.c_str(), controller_option_pair.first.isRos_?"true":"false", controller_option_pair.first.parameterPath_.c_str());
        }
        else
        {
          MELO_WARN("[RocomaRos] Subentry 'controller' of controllerpair no %d has missing or wrong-type entries. Skip controller.", i);
          continue;
        }

        // Parse emergency stop controller
        if(controller_pair_list[i]["controller_pair"].hasMember("emergency_controller") &&
            controller_pair_list[i]["controller_pair"]["emergency_controller"].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          controller = controller_pair_list[i]["controller_pair"]["emergency_controller"];
        }
        else
        {
          MELO_INFO("[RocomaRos] Controllerpair no %d has no member emergency_controller. Add failproof controller instead.", i);
          controller_option_pair.second = ManagedControllerOptions();
          controller_option_pairs.push_back(controller_option_pair);
          continue;
        }

        if( controller.hasMember("plugin_name") &&
            controller["plugin_name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            controller.hasMember("name") &&
            controller["name"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            controller.hasMember("is_ros") &&
            controller["is_ros"].getType() == XmlRpc::XmlRpcValue::TypeBoolean &&
            controller.hasMember("parameter_package") &&
            controller["parameter_package"].getType() == XmlRpc::XmlRpcValue::TypeString &&
            controller.hasMember("parameter_path") &&
            controller["parameter_path"].getType() == XmlRpc::XmlRpcValue::TypeString)
        {
          controller_option_pair.second.pluginName_ = static_cast<std::string>(controller["plugin_name"]);
          controller_option_pair.second.name_ = static_cast<std::string>(controller["name"]);
          controller_option_pair.second.isRos_ = static_cast<bool>(controller["is_ros"]);
          controller_option_pair.second.parameterPath_ = ros::package::getPath( static_cast<std::string>(controller["parameter_package"]) ) + "/" +
              static_cast<std::string>(controller["parameter_path"]);
          XmlRpc::XmlRpcValue shared_modules;
          controller_option_pair.second.sharedModuleNames_.clear();
          if(controller.hasMember("shared_modules") && controller["shared_modules"].getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int j = 0; j < controller["shared_modules"].size(); ++j) {
              if(controller["shared_modules"][j].getType() == XmlRpc::XmlRpcValue::TypeString) {
                controller_option_pair.second.sharedModuleNames_.push_back(controller["shared_modules"][j]);
              }
            }
          }
          MELO_INFO("[RocomaRos] Got controller plugin %s with controller name %s successfully from the parameter server.\n(is_ros: %s, complete parameter_path: %s!",
                    controller_option_pair.second.name_.c_str(), controller_option_pair.second.name_.c_str(), controller_option_pair.second.isRos_?"true":"false", controller_option_pair.second.parameterPath_.c_str());
        }
        else
        {
          MELO_WARN("[RocomaRos] Subentry 'emergency_controller' of controllerpair no %d has missing or wrong-type entries. Add failproof controller instead.", i);
          controller_option_pair.second = ManagedControllerOptions();
        }

        controller_option_pairs.push_back(controller_option_pair);
      }
    }
    else {
      MELO_WARN("[RocomaRos] Parameter 'controller_manager/controller_pairs' is not of array type. Add only failproof controller.");
      controller_option_pairs.clear();
    }
  }


  return setupControllers(failproofControllerName,
                          controller_option_pairs,
                          state,
                          command,
                          mutexState,
                          mutexCommand);

}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::emergencyStopService(std_srvs::Trigger::Request& req,
                                                                 std_srvs::Trigger::Response& res) {
  res.success = this->emergencyStop();
  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::failproofStopService(std_srvs::Trigger::Request& req,
                                                                 std_srvs::Trigger::Response& res) {
  res.success = this->failproofStop();
  return true;
}


template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::clearEmergencyStopService(std_srvs::Trigger::Request& req,
                                                                      std_srvs::Trigger::Response& res) {
  this->clearEmergencyStop();
  res.success = true;
  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::switchControllerService(rocoma_msgs::SwitchController::Request& req,
                                                                    rocoma_msgs::SwitchController::Response& res) {

  // This is another ros-thread anyway so this operation can be blocking until controller switched
  switch(  this->switchController(req.name) ) {
    case rocoma::ControllerManager::SwitchResponse::ERROR:
      res.status = res.STATUS_ERROR;
      break;
    case rocoma::ControllerManager::SwitchResponse::NOTFOUND:
      res.status = res.STATUS_NOTFOUND;
      break;
    case rocoma::ControllerManager::SwitchResponse::RUNNING:
      res.status = res.STATUS_RUNNING;
      break;
    case rocoma::ControllerManager::SwitchResponse::SWITCHING:
      res.status = res.STATUS_SWITCHED;
      break;
    case rocoma::ControllerManager::SwitchResponse::NA:
      res.status = res.STATUS_NA;
      break;
  }

  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::getAvailableControllersService( rocoma_msgs::GetAvailableControllers::Request& req,
                                                                            rocoma_msgs::GetAvailableControllers::Response& res) {
  res.available_controllers = this->getAvailableControllerNames();
  return true;
}

template<typename State_, typename Command_>
bool ControllerManagerRos<State_,Command_>::getActiveControllerService(rocoma_msgs::GetActiveController::Request& req,
                                                                       rocoma_msgs::GetActiveController::Response& res) {
  res.active_controller = this->getActiveControllerName();
  return true;
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::notifyEmergencyStop(rocoma::ControllerManager::EmergencyStopType type) {
  publishEmergencyState(true);
  publishEmergencyState(false);
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::notifyControllerManagerStateChanged(rocoma::ControllerManager::State state, bool emergencyStopCleared) {
  publishControllerManagerState(state, emergencyStopCleared);
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::notifyControllerChanged(const std::string & newControllerName) {
  publishActiveController(newControllerName);
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::publishActiveController(
    std::string activeController) {
  // Fill msg
  activeControllerMsg_.data = activeController;

  // Publish message
  activeControllerPublisher_.publish(activeControllerMsg_);
};

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::publishControllerManagerState(State state, bool emergencyStopCleared) {
  // Fill msg
  controllerManagerStateMsg_.stamp = ros::Time::now();
  controllerManagerStateMsg_.state = static_cast<int8_t>(state);
  controllerManagerStateMsg_.estop_cleared = emergencyStopCleared;

  // Publish message
  controllerManagerStatePublisher_.publish( controllerManagerStateMsg_ );
}

template<typename State_, typename Command_>
void ControllerManagerRos<State_,Command_>::publishEmergencyState(bool trigger) {
  // Fill msg
  emergencyStopStateMsg_.stamp = ros::Time::now();
  emergencyStopStateMsg_.trigger = trigger;

  // Publish message
  emergencyStopStatePublisher_.publish( emergencyStopStateMsg_ );
}

}
