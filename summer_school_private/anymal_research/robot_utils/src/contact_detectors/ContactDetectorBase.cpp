/*
 * ContactDetectorBase.cpp
 *
 *  Created on: Jan 25, 2017
 *      Author: Christian Gehring
 */

#include "robot_utils/contact_detectors/ContactDetectorBase.hpp"

namespace robot_utils {

ContactDetectorBase::ContactDetectorBase(const std::string& name) :
    name_(name),
    state_(ContactState::OPEN),
    wrench_()
{


}

ContactDetectorBase::~ContactDetectorBase() {

}

} /* namespace state_estimator */
