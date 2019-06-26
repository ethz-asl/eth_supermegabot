/*
 * rbdl_utils.cpp
 *
 *  Created on: Oct 1, 2015
 *      Author: Ralf Kaestner, Dario Bellicoso
 */

// romo
#include <romo_rbdl/rbdl_utils.hpp>

// melo
#include <message_logger/message_logger.hpp>

// stl
#include <assert.h>
#include <iostream>
#include <fstream>
#include <map>
#include <stack>
#include <list>
#include <algorithm>

// urdf
#include <urdf_model/model.h>

// Typedefs.
typedef urdf::LinkSharedPtr           LinkPtr;
typedef urdf::JointSharedPtr          JointPtr;
typedef urdf::ModelInterfaceSharedPtr ModelPtr;

typedef std::vector<LinkPtr>            URDFLinkVector;
typedef std::vector<JointPtr>           URDFJointVector;
typedef std::map<std::string, LinkPtr>  MapLinkNameToPtr;
typedef std::map<std::string, JointPtr> MapJointNameToPtr;
typedef std::map<std::string, std::list<std::string>::iterator> MapJointNameToJointList;

// Namespaces.
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics;

namespace romo {

namespace internal {
namespace console_colors {
  //colored strings
  const std::string black   = "\033[0;30m";
  const std::string red     = "\033[0;31m";
  const std::string green   = "\033[0;32m";
  const std::string yellow  = "\033[0;33m";
  const std::string blue    = "\033[0;34m";
  const std::string magenta = "\033[0;35m";
  const std::string cyan    = "\033[0;36m";
  const std::string white   = "\033[0;37m";
  const std::string def     = "\033[0m";
} /* namespace console_colors */

const Eigen::IOFormat eigenFormat(2, 0, ",","\n", "[", "]");

const std::map<RigidBodyDynamics::JointType, std::string> mapRbdlJointTypeToName = {
   {RigidBodyDynamics::JointType::JointTypeEulerZYX, "EulerZyx"},
   {RigidBodyDynamics::JointType::JointTypeFixed, "Fixed"},
   {RigidBodyDynamics::JointType::JointTypePrismatic, "Prismatic"},
   {RigidBodyDynamics::JointType::JointTypeRevolute, "Revolute"},
   {RigidBodyDynamics::JointType::JointTypeSpherical, "Spherical"},
   {RigidBodyDynamics::JointType::JointTypeTranslationXYZ, "TranslationXyz"}
};

} /* namespace internal */


bool construct_model(Model* rbdl_model, ModelPtr urdf_model,
                     const std::vector<std::string>& movable_joints_in_order,
                     bool verbose, bool useQuaternion) {

  const MapLinkNameToPtr&  mapLinkNameToPtr  = urdf_model->links_;
  const MapJointNameToPtr& mapJointNameToPtr = urdf_model->joints_;

  // Printout some info on the parser setup.
  if (verbose) {
    MELO_INFO_STREAM("[romo::construct_model] Build up to a model parsing a URDF.");
    MELO_INFO_STREAM("[romo::construct_model] Will use quaternions: " << (useQuaternion ? "yes" : "no"));
    MELO_INFO_STREAM("[romo::construct_model] The URDF contains the following links (shown as names):");
    for (const auto& pair : mapLinkNameToPtr) {
      std::cout << internal::console_colors::magenta << " ["
                << internal::console_colors::cyan << pair.first
                << internal::console_colors::magenta << "] ";
    }
    std::cout << internal::console_colors::def << std::endl;

    MELO_INFO_STREAM("[romo::construct_model] The URDF contains the following joints (shown as names):");
    for (const auto& pair : mapJointNameToPtr) {
      std::cout << internal::console_colors::magenta << " ["
                << internal::console_colors::cyan << pair.first
                << internal::console_colors::magenta << "] ";
    }
    std::cout << internal::console_colors::def << std::endl;

    MELO_INFO_STREAM("[romo::construct_model] The URDF contains the following topology ([parent] - (joint) - [child]):");
    for (const auto& pair : mapJointNameToPtr) {
      std::cout << internal::console_colors::magenta << "["
                << internal::console_colors::cyan << pair.second->parent_link_name
                << internal::console_colors::magenta << "]"
                << internal::console_colors::def << " - "
                << internal::console_colors::magenta << "("
                << internal::console_colors::cyan << pair.first
                << internal::console_colors::magenta << ")"
                << internal::console_colors::def << " - "
                << internal::console_colors::magenta << "["
                << internal::console_colors::cyan << pair.second->child_link_name
                << internal::console_colors::magenta << "]" << std::endl;
    }
    std::cout << internal::console_colors::def << std::endl;

    MELO_INFO_STREAM("[romo::construct_model] The movable joints will be reordered as (shown as [parent] - (joint) - [child]):");
    for (const auto& name : movable_joints_in_order) {

      auto jnt = mapJointNameToPtr.find(name);
      if (jnt != mapJointNameToPtr.end()) {
        std::cout << internal::console_colors::magenta << "["
                  << internal::console_colors::cyan << jnt->second->parent_link_name
                  << internal::console_colors::magenta << "]"
                  << internal::console_colors::def << " - "
                  << internal::console_colors::magenta << "("
                  << internal::console_colors::cyan << name
                  << internal::console_colors::magenta << ")"
                  << internal::console_colors::def << " - "
                  << internal::console_colors::magenta << "["
                  << internal::console_colors::cyan << jnt->second->child_link_name
                  << internal::console_colors::magenta << "]" << std::endl;
      } else {
        std::cout << "Joint named '" << name << "' could not be found in the joint map!" << std::endl;
      }

    }
    std::cout << internal::console_colors::def << std::endl;

  }

  // Get a reference to the root body.
  const urdf::LinkConstSharedPtr& rootLink = urdf_model->getRoot();

  if (verbose) {
    MELO_INFO_STREAM("Root body has name: " << internal::console_colors::cyan << rootLink->name << internal::console_colors::def);
  }


  /***************************
   * Setup the rootLink body *
   ***************************/
  // Check if the root body is the world or not.
  const bool worldLinkIsRoot = (rootLink->name == "world") ? true : false;
  const std::string rootBodyName = worldLinkIsRoot ? rootLink->child_links[0]->name
                                                   : rootLink->name;

  /* Attempt to find the link which contains the inertial data of the root body.
   * If the root body is called "base", then we assume the root inertial data will be in a
   * link called "base_inertia".
   */
  const MapLinkNameToPtr::const_iterator rootNameToPtrPairIterator = mapLinkNameToPtr.find(rootBodyName + "_inertia");
  if (rootNameToPtrPairIterator == mapLinkNameToPtr.end()) {
    std::string errorMsg = "[romo::construct_model] When searching for root body, could not find body with name: ";
    errorMsg.append(rootBodyName + "_inertia\n");
    MELO_ERROR_STREAM(errorMsg);
    return false;
  } else if (verbose) {
    MELO_INFO_STREAM("[romo::construct_model] Found root body inertial data link: "
        << internal::console_colors::cyan << rootBodyName + "_inertia"
        << internal::console_colors::def);
  }
  const urdf::LinkConstSharedPtr rootInertialDataLink = rootNameToPtrPairIterator->second;

  if (rootInertialDataLink->inertial) {
    /* The floating base root body will have two bodies:
     *  1. a virtual "null" body, which acts as a placeholder for the virtual translational joints;
     *  2. a body containing the full inertial data, which is connected to the "null" body by a 3Dof rotational joint parametrized
     *     by unit quaternions or ZYX Euler angles.
     */

    // Define the null translation body.
    const Matrix3d zeroMatrix = RigidBodyDynamics::Math::Matrix3d::Zero();
    Body nullBody = Body(0.0, Math::Vector3d::Zero(3), zeroMatrix);
    nullBody.mIsVirtual = true;


    // Define the orientation body.
    // Mass.
    const double rootBodyMass = rootInertialDataLink->inertial->mass;

    // Center of mass.
    Math::Vector3d rootBodyCom;
    rootBodyCom.set(rootInertialDataLink->inertial->origin.position.x,
                    rootInertialDataLink->inertial->origin.position.y,
                    rootInertialDataLink->inertial->origin.position.z);

    // Inertia matrix.
    Matrix3d rootBodyInertiaMatrix;
    rootBodyInertiaMatrix(0,0) = rootInertialDataLink->inertial->ixx;
    rootBodyInertiaMatrix(0,1) = rootInertialDataLink->inertial->ixy;
    rootBodyInertiaMatrix(0,2) = rootInertialDataLink->inertial->ixz;

    rootBodyInertiaMatrix(1,0) = rootInertialDataLink->inertial->ixy;
    rootBodyInertiaMatrix(1,1) = rootInertialDataLink->inertial->iyy;
    rootBodyInertiaMatrix(1,2) = rootInertialDataLink->inertial->iyz;

    rootBodyInertiaMatrix(2,0) = rootInertialDataLink->inertial->ixz;
    rootBodyInertiaMatrix(2,1) = rootInertialDataLink->inertial->iyz;
    rootBodyInertiaMatrix(2,2) = rootInertialDataLink->inertial->izz;

    const Body rbdlRootBody = Body(rootBodyMass,
                                   rootBodyCom,
                                   rootBodyInertiaMatrix);

    SpatialTransform rbdlRootBodyTransform = SpatialTransform ();



    if(worldLinkIsRoot){
      //assume fixed root body
      auto rootlinkParentJoint = mapLinkNameToPtr.find(rootBodyName)->second->parent_joint->parent_to_joint_origin_transform;
      Quaternion rotation(rootlinkParentJoint.rotation.x,
                          rootlinkParentJoint.rotation.y,
                          rootlinkParentJoint.rotation.z,
                          rootlinkParentJoint.rotation.w);
      rbdlRootBodyTransform.r = Vector3d(rootlinkParentJoint.position.x,
                                         rootlinkParentJoint.position.y,
                                         rootlinkParentJoint.position.z);
      rbdlRootBodyTransform.E = rotation.toMatrix();
      const Joint rbdlRootJointFixed(JointTypeFixed);
      // Append the translational and rotational bodies.
      rbdl_model->AppendBody(rbdlRootBodyTransform, rbdlRootJointFixed, nullBody);
      rbdl_model->AppendBody(rbdlRootBodyTransform,
                             rbdlRootJointFixed,
                             rbdlRootBody,
                             rootBodyName);
    }else{
      // Define the floating base joints.
      const Joint rbdlRootJointTranslation(JointTypeTranslationXYZ, "BASE_TRANS");
      const Joint rbdlRootJointRotation = useQuaternion ? Joint(JointTypeSpherical, "BASE_ROT")
                                                        : Joint(JointTypeEulerZYX, "BASE_ROT");
      // Append the translational and rotational bodies.
      rbdl_model->AppendBody(rbdlRootBodyTransform, rbdlRootJointTranslation, nullBody);
      rbdl_model->AppendBody(rbdlRootBodyTransform,
                             rbdlRootJointRotation,
                             rbdlRootBody,
                             rootBodyName);
    }

    // Set the root id to the non-virtual body.
    rbdl_model->SetRootIdByName(rootBodyName);
  } else {
    MELO_ERROR_STREAM("[romo::construct_model] Root inertial had no inertial data.");
    return false;
  }
  /***********************/

  if (verbose) {
    const unsigned int rbdlRootBodyId = rbdl_model->GetBodyId(rootBodyName.c_str());
    MELO_INFO_STREAM("[romo::construct_model] Added a floating base body to the model.");
    std::cout << "String name: " << rootBodyName.c_str() << std::endl;
    if (rbdl_model->IsFixedBodyId(rbdlRootBodyId)) {
      std::cout << "Body is fixed." << std::endl;
    } else {
      std::cout << "Id: "   << rbdlRootBodyId << std::endl;
      std::cout << "Name: " << rbdl_model->GetBodyName(rbdlRootBodyId) << std::endl;
      std::cout << "Mass: " << rbdl_model->mBodies[rbdlRootBodyId].mMass << std::endl;
      std::cout << "CoM:  " << rbdl_model->mBodies[rbdlRootBodyId].mCenterOfMass.transpose().format(internal::eigenFormat) << std::endl;
      std::cout << "Inertia:" << std::endl << rbdl_model->mBodies[rbdlRootBodyId].mInertia.format(internal::eigenFormat) << std::endl;
    }
  }


  /***********************/


  // Reorder joint pointers according to the movable_joints_in_order input.
  MapJointNameToJointList mapMovableJointNameToOrderedJointNameListItem;
  MapJointNameToJointList mapJointNameToOrderedJointNameListItem;
  std::list<std::string> orderedJointNamesList;
  for (unsigned int k=0; k<movable_joints_in_order.size(); ++k) {
    orderedJointNamesList.push_back(movable_joints_in_order[k]);
    mapMovableJointNameToOrderedJointNameListItem.insert(std::make_pair(movable_joints_in_order[k],
                                                                        --orderedJointNamesList.end()));
    mapJointNameToOrderedJointNameListItem.insert(std::make_pair(movable_joints_in_order[k],
                                                                 --orderedJointNamesList.end()));
  }
  if (verbose) {
    std::cout << std::endl;
    std::cout << "Joint names have been ordered as:" << std::endl;
    for (const auto& name : orderedJointNamesList) {
      std::cout << internal::console_colors::magenta << " ["
                << internal::console_colors::cyan << name
                << internal::console_colors::magenta << "] ";
    }
    std::cout << internal::console_colors::def << std::endl;
  }

  /*
   * To ensure all bodies are added AND the joints are in the correct order,
   * two iterations are performed:
   *   1. For all movable joints, trace the links/joints back to the
   *      root body and add all intermediate joints.
   *   2. Scan the complete model tree in a depth-first manner to add
   *      remaining links/joints that were missed before.
   */

  /*
   * Iteration 1: Trace back from each movable joint.
   */

  //make a copy of the movable_joints_in_order which we can safely modify
  std::vector<std::string> movable_joints_in_order_cpy = movable_joints_in_order;
  for (unsigned int i=0; i<movable_joints_in_order_cpy.size(); i++){
    const std::string movableJointName = movable_joints_in_order_cpy.at(i);
    const std::string parentLinkName = mapJointNameToPtr.at(movableJointName)->parent_link_name;

    if(parentLinkName != rootBodyName){
      //since the parent link is not the root body of our robot, it must have a parent joint
      //this is the parent joint of the current movable joint
      std::string parentJointName = mapLinkNameToPtr.at(parentLinkName)->parent_joint->name;
      if(std::find(movable_joints_in_order_cpy.begin(), movable_joints_in_order_cpy.end(), parentJointName) == movable_joints_in_order_cpy.end()){
        //The parent joint is not in our list yet -> Add it to inspect it (we will loop through it later)
        movable_joints_in_order_cpy.push_back(parentJointName);

        /*
         * Finally, insert parentJointName into the orderedJointNamesList before the current joint.
         * The current joint is guaranteed to be in orderedJointNamesList, because it was either
         * added by us already (here) or it was there from the beginning (in case it's movable)
         */
        auto it = orderedJointNamesList.insert(mapJointNameToOrderedJointNameListItem.at(movableJointName), parentJointName);
        mapJointNameToOrderedJointNameListItem.insert(std::make_pair(parentJointName, it));
      }
    }
  }

  if (verbose) {
    std::cout << std::endl;
    std::cout << "After iteration 1, joint names have been modified as:" << std::endl;
    for (const auto& name : orderedJointNamesList) {
      std::cout << internal::console_colors::magenta << " ["
                << internal::console_colors::cyan << name
                << internal::console_colors::magenta << "] ";
    }
    std::cout << internal::console_colors::def << std::endl;
  }

  /*
   * Iteration 2: Add the bodies in a depth-first order of the model tree.
   */
  std::stack<LinkPtr> linkStack;
  linkStack.push(mapLinkNameToPtr.at((rootLink->name)));


  std::stack<int> jointIndexStack;
  if (linkStack.top()->child_joints.size() > 0) {
    if (verbose) {
      const LinkPtr topLink = linkStack.top();
      const std::vector<JointPtr> childJoints = topLink->child_joints;
      std::cout << "The top link in the stack (" << topLink->name << ") has " << topLink->child_joints.size() << " children joints:" << std::endl;
      for (const auto& joint : childJoints) {
        std::cout << internal::console_colors::magenta << " ["
                  << internal::console_colors::cyan << joint->name
                  << internal::console_colors::magenta << "] ";
      }
      std::cout << internal::console_colors::def << std::endl;
    }
    jointIndexStack.push(0);
  }


  while (linkStack.size() > 0) {
    const LinkPtr linkStackTop = linkStack.top();
    const unsigned int jointIndexStackTop = jointIndexStack.top();

    if (verbose) {
      std::cout << std::endl;
      std::cout << "Current link: " << linkStackTop->name << std::endl;
      std::cout << "Joint idx: " << jointIndexStackTop << std::endl;
    }

    if (jointIndexStackTop < linkStackTop->child_joints.size()) {
      if (verbose) {
        std::cout << "There are still links to parse in the current branch. Joint idx: " << jointIndexStackTop << ". Child joints size: " << linkStackTop->child_joints.size() << std::endl;
      }
      const JointPtr cur_joint = linkStackTop->child_joints[jointIndexStackTop];

      // Increment joint index.
      jointIndexStack.pop();
      jointIndexStack.push(jointIndexStackTop + 1);

      linkStack.push(mapLinkNameToPtr.at(cur_joint->child_link_name));
      jointIndexStack.push(0);

      //find out if current joint is mobile and if it's in the list already
      const MapJointNameToJointList::iterator currentJointIt = mapJointNameToOrderedJointNameListItem.find(cur_joint->name);
      const MapJointNameToJointList::iterator currentMovableJointIt = mapMovableJointNameToOrderedJointNameListItem.find(cur_joint->name);

      // if the current joint is a movable one, there's nothing to do
      if (currentMovableJointIt == mapMovableJointNameToOrderedJointNameListItem.end()) {
        // if a joint is not in the robot model enum list, set it as a fixed or mimiced joint
        if (mapJointNameToPtr.at(cur_joint->name)->mimic == nullptr) {
          mapJointNameToPtr.at(cur_joint->name)->type = urdf::Joint::FIXED;
        // if a joint is not in the robot model enum list but is a mimic, check if the mimic root is in the enum list
        } else {
          JointPtr parent_joint = mapJointNameToPtr.at(cur_joint->mimic->joint_name);
          // travel up the mimic tree until the non-mimic (I.E. controlled) joint is found
          while(parent_joint->mimic!=nullptr){
            const std::string new_parent_name = parent_joint->mimic->joint_name;
            parent_joint = mapJointNameToPtr.at(new_parent_name);
          }
          // if the mimic root of a joint is not in the list, set it as fixed
          const MapJointNameToJointList::iterator mimicRootJointIt =
              mapMovableJointNameToOrderedJointNameListItem.find(parent_joint->name);
          if (mimicRootJointIt == mapMovableJointNameToOrderedJointNameListItem.end()){
            mapJointNameToPtr.at(cur_joint->name)->type = urdf::Joint::FIXED;
          }
        }
        //now if joint is not already present in orderedJointNamesList, just add it at the end
        if (currentJointIt == mapJointNameToOrderedJointNameListItem.end()){
          orderedJointNamesList.push_back(cur_joint->name);
          mapJointNameToOrderedJointNameListItem.insert(std::make_pair(cur_joint->name,
                                                                       --orderedJointNamesList.end()));
          if (verbose) {
            std::cout << "Joint " << internal::console_colors::cyan << cur_joint->name << internal::console_colors::def
                      << " has been added to joint names." << std::endl;
          }
        } else {
          if (verbose) {
            std::cout << "Joint " << internal::console_colors::cyan << cur_joint->name << internal::console_colors::def
                      << " is not movable but already in joint names. Not adding again." << std::endl;
          }
        }
      } else {
        if (verbose) {
          std::cout << "Joint " << internal::console_colors::cyan << cur_joint->name << internal::console_colors::def
                    << " is movable and already in joint names. Not adding again." << std::endl;
        }
      }

      if (verbose) {
        std::stack<int> jointIndexStackCopy = jointIndexStack;

        std::cout << "Joint index stack:" << std::endl;
        for (; !jointIndexStackCopy.empty(); jointIndexStackCopy.pop()) {
          std::cout << internal::console_colors::magenta << " ["
                    << internal::console_colors::cyan << jointIndexStackCopy.top()
                    << internal::console_colors::magenta << "] ";
        }
        std::cout << internal::console_colors::def << std::endl;
      }



    } else {
      //done with this branch, no more child joints.
      if (verbose) {
        std::cout << "Done parsing joints in the current branch." << std::endl;
        std::cout << "Link '"  << linkStack.top()->name << "' will be popped from stack." << std::endl;
        std::cout << "Joint idx '"  << jointIndexStack.top() << "' will be popped from stack." << std::endl;
      }

      linkStack.pop();
      jointIndexStack.pop();
    }

  } // while


  if (verbose) {
    std::cout << std::endl;
    std::cout << "After iteration 2, joint names have been modified as:" << std::endl;
    for (const auto& name : orderedJointNamesList) {
      std::cout << internal::console_colors::magenta << " ["
                << internal::console_colors::cyan << name
                << internal::console_colors::magenta << "] ";
    }
    std::cout << internal::console_colors::def << std::endl;
  }
  /*************************************************************************/

  // Joints and links have all been parsed, now add them to the model.
  // The root body has been added at the beginning of this method.
  for (std::list<std::string>::const_iterator it = orderedJointNamesList.begin(); it != orderedJointNamesList.end(); it++) {
    const JointPtr urdf_joint = mapJointNameToPtr.at(*it);
    const LinkPtr urdf_parent = mapLinkNameToPtr.at(urdf_joint->parent_link_name);
    const LinkPtr urdf_child = mapLinkNameToPtr.at(urdf_joint->child_link_name);

    if (verbose) {
      std::cout << std::endl;
      MELO_INFO_STREAM("[romo::construct_model] ****************************************");
      MELO_INFO_STREAM("[romo::construct_model] Current joint name: " << urdf_joint->name);
      MELO_INFO_STREAM("[romo::construct_model] Parent link: " << urdf_parent->name);
      MELO_INFO_STREAM("[romo::construct_model] Child link: " << urdf_child->name);
    }

    // Ignore the root inertial joint (this link was already added).
    if (urdf_child->name == rootInertialDataLink->name) {
      if (verbose) {
        MELO_INFO_STREAM("[romo::construct_model] Child link is inertial data link. Skipping joint.");
        MELO_INFO_STREAM("[romo::construct_model] ****************************************");
      }
      continue;
    }

    if (worldLinkIsRoot && urdf_parent->name == "world") {
      if (verbose) {
        MELO_INFO_STREAM("[romo::construct_model] Parent link is world. Skipping joint.");
        MELO_INFO_STREAM("[romo::construct_model] ****************************************");
      }
      continue;
    }

    // Determine where to add the current joint and child body. Join bodies with root body instead of the root inertial body.
    unsigned int rbdl_parent_id = 0;
    if (urdf_parent->name != rootInertialDataLink->name) {
      rbdl_parent_id = rbdl_model->GetBodyId(urdf_parent->name.c_str());
    } else if (!worldLinkIsRoot) {
      rbdl_parent_id = rbdl_model->GetBodyId(rootLink->name.c_str());
    } else {
      rbdl_parent_id = rbdl_model->GetBodyId(rootLink->child_links[0]->name.c_str());
      std::cout << "World link is root. Connecting with: " << rootLink->child_links[0] << std::endl;
    }

    if (rbdl_parent_id == std::numeric_limits<unsigned int>::max()) {
      std::cerr << "Error while processing joint '" << urdf_joint->name
                << "': parent link '" << urdf_parent->name
                << "' could not be found." << std::endl;
    }

    //cout << "joint: " << urdf_joint->name << "\tparent = " << urdf_parent->name << " child = " << urdf_child->name << " parent_id = " << rbdl_parent_id << std::endl;

    /********************************
     * Create the appropriate joint *
     ********************************/
    Joint rbdl_joint;
    urdf::JointMimic mimic_joint;
    if(urdf_joint->mimic.get() == nullptr) {
      mimic_joint.joint_name.clear();
      mimic_joint.multiplier = 1.0;
      mimic_joint.offset = 0.0;
    }
    else {
      mimic_joint = *urdf_joint->mimic;
    }

    switch (urdf_joint->type) {
      case(urdf::Joint::REVOLUTE):
      case(urdf::Joint::CONTINUOUS): {
        if (verbose) {
          MELO_INFO_STREAM("[romo::construct_model] Joint is movable.");
        }
        rbdl_joint = Joint(SpatialVector(urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z, 0., 0., 0.),
        					urdf_joint->name,
                  mimic_joint.joint_name,
                  mimic_joint.multiplier,
                  mimic_joint.offset);
      } break;
      case(urdf::Joint::PRISMATIC): {
        if (verbose) {
          MELO_INFO_STREAM("[romo::construct_model] Joint is movable.");
        }
        rbdl_joint = Joint(SpatialVector(0., 0., 0.,urdf_joint->axis.x, urdf_joint->axis.y, urdf_joint->axis.z),
							urdf_joint->name,
              mimic_joint.joint_name,
							mimic_joint.multiplier,
							mimic_joint.offset);
      } break;
      case(urdf::Joint::FIXED): {
        if (verbose) {
          MELO_INFO_STREAM("[romo::construct_model] Joint is fixed.");
        }
        rbdl_joint = Joint (JointTypeFixed,
              urdf_joint->name,
							mimic_joint.joint_name,
							mimic_joint.multiplier,
							mimic_joint.offset);
      } break;
      case(urdf::Joint::FLOATING): {
        rbdl_joint = Joint(SpatialVector(0., 0., 0., 1., 0., 0.),
                           SpatialVector(0., 0., 0., 0., 1., 0.),
                           SpatialVector(0., 0., 0., 0., 0., 1.),
                           SpatialVector(0., 0., 1., 0., 0., 0.),
                           SpatialVector(0., 1., 0., 0., 0., 0.),
                           SpatialVector(1., 0., 0., 0., 0., 0.),
						   urdf_joint->name,
               mimic_joint.joint_name,
						   mimic_joint.multiplier,
						   mimic_joint.offset);
      } break;
      case(urdf::Joint::PLANAR): {
        std::cerr << "Error while processing joint '" << urdf_joint->name << "': planar joints not yet supported!" << std::endl;
        return false;
      } break;
      default: {
        std::cerr << "Error while processing joint '" << urdf_joint->name << "': unsupported joint type " << urdf_joint->type << "!" << std::endl;
        return false;
      }
    }
    /********************************/


    /*****************************************************************************
     * Compute the pose transformation from the parent joint to the current one. *
     *****************************************************************************/
    Math::Vector3d joint_rpy;
    Math::Vector3d joint_translation;

    // Get the rotation w.r.t. the parent joint.
    urdf_joint->parent_to_joint_origin_transform.rotation.getRPY(joint_rpy[0], joint_rpy[1], joint_rpy[2]);

    // Get the translation w.r.t. the parent joint.
    joint_translation.set (urdf_joint->parent_to_joint_origin_transform.position.x,
                           urdf_joint->parent_to_joint_origin_transform.position.y,
                           urdf_joint->parent_to_joint_origin_transform.position.z);
    // Compute the joint transformation.
    const SpatialTransform rbdl_joint_frame =
          Xrot (static_cast<double>(joint_rpy[0]), Math::Vector3d (1., 0., 0.))
        * Xrot (static_cast<double>(joint_rpy[1]), Math::Vector3d (0., 1., 0.))
        * Xrot (static_cast<double>(joint_rpy[2]), Math::Vector3d (0., 0., 1.))
        * Xtrans (Math::Vector3d(joint_translation));

    if (verbose) {
      MELO_INFO_STREAM("[romo::construct_model] Position parent to joint origin          : " << joint_translation.transpose());
      MELO_INFO_STREAM("[romo::construct_model] Orientation child link w.r.t. parent(rpy): " << joint_rpy.transpose());
    }
    /*****************************************************************************/

    /********************************
     * Parse the link inertial data *
     ********************************/
    Math::Vector3d link_inertial_position = Math::Vector3d::Zero();
    Math::Vector3d link_inertial_rpy = Math::Vector3d::Zero();
    Matrix3d link_inertial_inertia = Matrix3d::Zero();
    double link_inertial_mass = 0.0;

    // but only if we actually have inertial data
    if (urdf_child->inertial) {

      link_inertial_mass = urdf_child->inertial->mass;

      link_inertial_position.set(urdf_child->inertial->origin.position.x,
                                 urdf_child->inertial->origin.position.y,
                                 urdf_child->inertial->origin.position.z);

      urdf_child->inertial->origin.rotation.getRPY(link_inertial_rpy[0],
                                                   link_inertial_rpy[1],
                                                   link_inertial_rpy[2]);

      link_inertial_inertia(0,0) = urdf_child->inertial->ixx;
      link_inertial_inertia(0,1) = urdf_child->inertial->ixy;
      link_inertial_inertia(0,2) = urdf_child->inertial->ixz;

      link_inertial_inertia(1,0) = urdf_child->inertial->ixy;
      link_inertial_inertia(1,1) = urdf_child->inertial->iyy;
      link_inertial_inertia(1,2) = urdf_child->inertial->iyz;

      link_inertial_inertia(2,0) = urdf_child->inertial->ixz;
      link_inertial_inertia(2,1) = urdf_child->inertial->iyz;
      link_inertial_inertia(2,2) = urdf_child->inertial->izz;

      if (link_inertial_rpy != Math::Vector3d (0., 0., 0.)) {
        MELO_WARN_STREAM("While processing body '" << urdf_child->name
                         << "': rotation of body frames not yet supported. Please rotate the joint frame instead. "
                         << "Setting rotation to zero.");
        MELO_WARN_STREAM("link inertial rpy is !=0");
        link_inertial_rpy.set(0.0, 0.0, 0.0);
//        return false;
      }
      if (verbose) {
        MELO_INFO_STREAM("[romo::construct_model] Mass: " << link_inertial_mass);
        MELO_INFO_STREAM("[romo::construct_model] CoM:  " << link_inertial_position.transpose());
      }
    } else {
      if (verbose) {
        MELO_INFO_STREAM("[romo::construct_model] No inertial data for body: " << urdf_child->name);
      }
    }

    const Body rbdl_body = Body (link_inertial_mass, link_inertial_position, link_inertial_inertia);
    /********************************/

    if (verbose) {
      std::cout << "+ Adding Body " << std::endl;
      std::cout << "  parent_id        : " << rbdl_parent_id << std::endl;
      std::cout << "  joint frame      : " << std::endl;
      std::cout << " E:" << std::endl << rbdl_joint_frame.E.matrix().format(internal::eigenFormat) << std::endl;
      std::cout << " r:" << rbdl_joint_frame.r.transpose() << std::endl;
      std::cout << "  joint dofs count : " << rbdl_joint.mDoFCount << std::endl;
      for (unsigned int j = 0; j < rbdl_joint.mDoFCount; j++) {
        std::cout << "\taxes for dof " << j << ": " << rbdl_joint.mJointAxes[j].transpose() << std::endl;
      }
      std::cout << "  body inertia: " << std::endl << rbdl_body.mInertia.format(internal::eigenFormat) << std::endl;
      std::cout << "  body mass   : " << rbdl_body.mMass << std::endl;
      std::cout << "  body name   : " << urdf_child->name << std::endl;
    }

    int movableParentId = -1;
    if (urdf_joint->type == urdf::Joint::FIXED) {
      if (rbdl_model->IsFixedBodyId(rbdl_parent_id)) {
        if (verbose) {
          MELO_INFO_STREAM("[romo::construct_model] Parent is fixed. Parent Id: " << rbdl_parent_id);
        }
        movableParentId = rbdl_model->mFixedBodies[rbdl_parent_id - rbdl_model->fixed_body_discriminator].mMovableParent;
      } else {
        if (verbose) {
          MELO_INFO_STREAM("[romo::construct_model] Parent is movable.");
        }
        movableParentId = rbdl_parent_id;
      }
      if (verbose) {
        MELO_INFO_STREAM("[romo::construct_model] Movable parent id: " << movableParentId);
        MELO_INFO_STREAM("[romo::construct_model] Movable parent name: " << rbdl_model->GetBodyName(movableParentId));
        MELO_INFO_STREAM("[romo::construct_model] Movable parent inertia before adding fixed joint: " << std::endl
                         << "mass: " << rbdl_model->mBodies[movableParentId].mMass << std::endl
                         << " com: " << rbdl_model->mBodies[movableParentId].mCenterOfMass.transpose());
      }
    }


    if (/*0 && */urdf_joint->type == urdf::Joint::FLOATING) {
      Matrix3d zero_matrix = Matrix3d::Zero();
      Body null_body (0., Math::Vector3d::Zero(3), zero_matrix);
      Joint joint_txtytz(JointTypeTranslationXYZ);
      std::string trans_body_name = urdf_child->name + "_Translate";
      rbdl_model->AddBody (rbdl_parent_id, rbdl_joint_frame, joint_txtytz, null_body, trans_body_name);

      Joint joint_euler_zyx (JointTypeEulerXYZ);
      rbdl_model->AppendBody (SpatialTransform(), joint_euler_zyx, rbdl_body, urdf_child->name);
    } else {
      rbdl_model->AddBody(rbdl_parent_id, rbdl_joint_frame, rbdl_joint, rbdl_body, urdf_child->name);
    }

    if (urdf_joint->type == urdf::Joint::FIXED) {
      if (verbose) {
        MELO_INFO_STREAM("[romo::construct_model] Movable parent inertia after adding fixed joint: " << std::endl
                         << "mass: " << rbdl_model->mBodies[movableParentId].mMass << std::endl
                         << " com: " << rbdl_model->mBodies[movableParentId].mCenterOfMass.transpose());
      }
    }

    if (verbose) {
      MELO_INFO_STREAM("[romo::construct_model] ****************************************");
    }

  } // for

  if (verbose) {
    MELO_INFO_STREAM("[romo::construct_model] Done constructing model.");
  }

  return true;
}

bool URDFReadFromFile(urdf::ModelInterfaceSharedPtr& urdf_model,
                      const char* filename,
                      Model* model,
                      const std::vector<std::string>& movable_joints_in_order,
                      bool verbose,
                      bool useQuaternion,
                      double absoluteGravityAcceleration) {
  std::ifstream model_file (filename);
  if (!model_file) {
    MELO_WARN_STREAM("[romo::URDFReadFromFile] Error opening file: " << filename);
    return false;
  }

  // reserve memory for the contents of the file
  std::string model_xml_string;
  model_file.seekg(0, std::ios::end);
  model_xml_string.reserve(model_file.tellg());
  model_file.seekg(0, std::ios::beg);
  model_xml_string.assign((std::istreambuf_iterator<char>(model_file)), std::istreambuf_iterator<char>());
  model_file.close();

  return URDFReadFromString (urdf_model, model_xml_string.c_str(), model, movable_joints_in_order, verbose, useQuaternion, absoluteGravityAcceleration);
}

bool URDFReadFromFile(const char* filename, RigidBodyDynamics::Model* model,
                      const std::vector<std::string>& movable_joints_in_order,
                      bool verbose,
                      bool useQuaternion,
                      double absoluteGravityAcceleration) {
  urdf::ModelInterfaceSharedPtr urdf_model;
  return URDFReadFromFile(urdf_model, filename,model, movable_joints_in_order, verbose, useQuaternion, absoluteGravityAcceleration);
}

bool URDFReadFromString(urdf::ModelInterfaceSharedPtr& urdf_model,
                        const char* model_xml_string,
                        Model* model,
                        const std::vector<std::string>& movable_joints_in_order,
                        bool verbose,
                        bool useQuaternion,
                        double absoluteGravityAcceleration) {
  assert (model);

  urdf_model = urdf::parseURDF (model_xml_string);

  if (!construct_model (model, urdf_model, movable_joints_in_order, verbose, useQuaternion)) {
    MELO_WARN_STREAM("[romo::URDFReadFromFile] Error constructing model from urdf file.");
    return false;
  }

  model->gravity.set (0., 0., -absoluteGravityAcceleration);

  return true;
}

bool URDFReadFromString(const char* model_xml_string,
                        RigidBodyDynamics::Model* model,
                        const std::vector<std::string>& movable_joints_in_order,
                        bool verbose,
                        bool useQuaternion,
                        double absoluteGravityAcceleration) {
  urdf::ModelInterfaceSharedPtr urdf_model;
  return URDFReadFromString(urdf_model, model_xml_string, model, movable_joints_in_order, verbose, useQuaternion, absoluteGravityAcceleration);
}

}
