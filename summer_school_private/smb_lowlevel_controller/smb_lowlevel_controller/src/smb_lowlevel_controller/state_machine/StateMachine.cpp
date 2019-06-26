// anymal lowlevel controller
#include "anymal_lowlevel_controller/state_machine/states.hpp"


namespace anymal_lowlevel_controller {
namespace state_machine {


StateMachine::StateMachine(const SystemPtr& system)
: Base(States({
           StatePtrType(new ActionActuatorsClearErrors(*this)),
           StatePtrType(new ActionActuatorsDisable(*this)),
           StatePtrType(new ActionActuatorsEnable(*this)),
           StatePtrType(new ActionActuatorsWarmReset(*this)),
           StatePtrType(new ActionGoDefault(*this)),
           StatePtrType(new ActionGoDock(*this)),
           StatePtrType(new ActionGoRest(*this)),
           StatePtrType(new ActionGoZero(*this)),
           StatePtrType(new StateFatal(*this)),
           StatePtrType(new StateOperational(*this)),
           StatePtrType(new StateIdle(*this)),
           StatePtrType(new StateZeroJointTorque(*this))}),
       Substates(
           AnydrivesStateEnum::NA),
       StateEnum::StateIdle),
  system_(system) {}

StateMachine::~StateMachine() {}

const SystemPtr& StateMachine::getSystem()
{
  return system_;
}


} // state_machine
} // anymal_lowlevel_controller
