/*!
 * @file     SmbModelRosConverter.cpp
 * @author   Koen Kraemer
 * @date     Aug, 2018
 */

#include "smb_model_ros/SmbModelRosConverter.hpp"
#include "smb_model_ros/SmbStateRosConverter.hpp"

using namespace smb_model;

namespace smb_model_ros {

    bool SmbModelRosConverter::fromMessage(const smb_msgs::SmbState& message,
                                            smb_model::SmbModel& model)
    {
      using RD = smb_description::SmbDescription;
      using RT = smb_description::SmbTopology;

      smb_model::SmbState state;
      SmbStateRosConverter::fromMessage(message, state);

//      JointTorques jointTorques;

//      for (const auto armActuatorKey : RD::getKeys<RT::KinovaActuatorEnum>()) {
//        const auto armActuatorEnum = armActuatorKey.getEnum();
//        const auto armJointId = armActuatorKey.getId();
//        jointTorques(armJointId) = message.joints.effort[armJointId];
//      }

//      model.setJointTorques(jointTorques);

      // Set state in model.
      model.setState(state, true, true, false);
      return true;
    }

    bool SmbModelRosConverter::toMessage(const smb_model::SmbModel& model,
                                          smb_msgs::SmbState& message)
    {
      return false;
    }

} /* namespace smb_model_ros */
