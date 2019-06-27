//! Adapter interfaces
#include "roco/controllers/adapters/ControllerAdapterInterface.hpp"
#include "roco/controllers/adapters/EmergencyControllerAdapterInterface.hpp"
#include "roco/controllers/adapters/FailproofControllerAdapterInterface.hpp"

//! Adaptee interfaces
#include "roco/controllers/adaptees/ControllerAdapteeInterface.hpp"
#include "roco/controllers/adaptees/EmergencyControllerAdapteeInterface.hpp"
#include "roco/controllers/adaptees/FailproofControllerAdapteeInterface.hpp"

//! Extension interface
#include "roco/controllers/ControllerExtensionInterface.hpp"

//! Controllers
#include "roco/controllers/ControllerBase.hpp"
#include "roco/controllers/Controller.hpp"
#include "roco/controllers/FailproofController.hpp"
#include "roco/controllers/ControllerTupleBase.hpp"
#include "roco/controllers/ControllerTuple.hpp"

//! Swap States
#include "roco/controllers/ControllerSwapStateInterface.hpp"
#include "roco/controllers/ControllerTupleSwapState.hpp"

//! Shared Modules
#include "roco/controllers/SharedModule.hpp"