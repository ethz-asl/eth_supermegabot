/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \author  Peter Pastor, Peter Fankhauser

 **********************************************************************/

// Local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/TransformationSystem.hpp"

// import most common Eigen types
//USING_PART_OF_NAMESPACE_EIGEN // TODO

// message logger
#include <message_logger/log/log_messages.hpp>

namespace dmp
{

//static const char* transformation_system_file_name = "trans_";
//static const char* transformation_system_topic_name = "transformation_system";

// Default Constructor. Note, everything need to be initialized since it is used in a vector inside DMPMotionUnit.
TransformationSystem::TransformationSystem(DiscreteMovementPrimitive* dmpParent) :
		dmpParent_(dmpParent), initialized_(false), z_(0), zd_(0), y_(0), yd_(0), ydd_(0), t_(0), td_(0), tdd_(0), y0_(0), initial_y0_(0), goal_(0),
        initial_goal_(0), f_(0), ft_(0), mse_(0), mean_ft_(0), num_mse_data_points_(0)
{
    gaussianKernelModel_ = new GaussianKernel();
}

TransformationSystem::~TransformationSystem()
{
	delete gaussianKernelModel_;
}

bool TransformationSystem::initialize(const int num_rfs, const double activation, const bool exponentially_spaced, const double can_sys_cutoff)
{
    if (initialized_)
    {
        printf("Transformation system already initialized. Reinitializing with new parameters.\n");
    }

    if (!gaussianKernelModel_->initialize(num_rfs, activation, exponentially_spaced, can_sys_cutoff))
    {
        printf("Could not initialize LWR model.\n");
        initialized_ = false;
        return initialized_;
    }

    trajectory_target_.clear();

    initialized_ = true;
    return initialized_;
}

void TransformationSystem::print()
{
    //MELO_INFO_STREAM(getInfoString());
}

std::string TransformationSystem::getInfoString()
{
	std::string info("");
    std::stringstream ss;

    int precision = 4;
    ss.precision(precision);
    ss << std::fixed;

    //ss << trans_id_;
    info.append(std::string("index: ") + ss.str());
    ss.str("");
    ss.clear();

    info.append(std::string("   initialized: "));
    if (initialized_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    if (initial_y0_ < 0)
    {
        ss.precision(precision - 1);
    }
    ss << initial_y0_;
    info.append(std::string("  initial start state: ") + ss.str());
    ss.str("");
    ss.clear();
    ss.precision(precision);

    if (initial_goal_ < 0)
    {
        ss.precision(precision - 1);
    }
    ss << initial_goal_;
    info.append(std::string("   initial goal state: ") + ss.str());
    ss.str("");
    ss.clear();
    ss.precision(precision);

    info.append(std::string("\n") + gaussianKernelModel_->getInfoString());
    return info;
}

}
