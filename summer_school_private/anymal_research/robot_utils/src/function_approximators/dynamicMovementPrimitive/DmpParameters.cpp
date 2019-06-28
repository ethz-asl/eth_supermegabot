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

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/DmpParameters.hpp"

namespace dmp
{

DmpParameters::DmpParameters() :
    is_learned_(false), is_setup_(false), is_start_set_(false), num_transformation_systems_(0), tau_(0), initial_tau_(0), delta_t_(0), initial_delta_t_(0),
            teaching_duration_(-1), execution_duration_(-1),
            can_sys_cutoff_(0), alpha_x_(-1), alpha_z_(-1), beta_z_(-1), sample_index_(0)
{
}

DmpParameters::~DmpParameters()
{
}

bool DmpParameters::initialize(const double sampling_frequency,
                               const double teaching_duration,
                               const double execution_duration,
                               const double can_sys_cutoff,
                               const double alpha_z,
                               const double beta_z)
{
    if (teaching_duration <= 0)
    {
        printf("Teaching duration is invalid (%.1f sec).", teaching_duration);
        return false;
    }

    if (execution_duration <= 0)
    {
        printf("Execution duration is invalid (%.1f sec).", execution_duration);
        return false;
    }

    if (can_sys_cutoff <= 0)
    {
        printf("Canonical system cutoff frequency is invalid (%.1f).", can_sys_cutoff);
        return false;
    }

    if (alpha_z <= 0)
    {
        printf("Time constant alpha_z is invalid (%f).", alpha_z);
        return false;
    }

    if (beta_z <= 0)
    {
        printf("Time constant beta_z is invalid (%f).", beta_z);
        return false;
    }

	teaching_duration_ = teaching_duration;
	execution_duration_ = execution_duration;
	can_sys_cutoff_ = can_sys_cutoff;
	alpha_z_ = alpha_z;
	beta_z_ = beta_z;
	default_sampling_frequency_ = sampling_frequency;

    return true;
}

void DmpParameters::print()
{
   // printf("%s", getInfoString());
}

std::string DmpParameters::getInfoString()
{
    std::string info("");
    std::stringstream ss;

    int precision = 3;
    ss.precision(precision);
    ss << std::fixed;

    ss << teaching_duration_;
    info.append(std::string("\t"));
    info.append(std::string("teaching duration: ") + ss.str());
    ss.str("");
    ss.clear();

    ss << execution_duration_;
    info.append(std::string("   execution duration: ") + ss.str());
    ss.str("");
    ss.clear();

    info.append(std::string("\n\t"));
    info.append(std::string("canonical system parameters>      "));

    ss << alpha_x_;
    info.append(std::string("  alpha_x: ") + ss.str());
    ss.str("");
    ss.clear();

    ss << can_sys_cutoff_;
    info.append(std::string("   canonical system cutoff: ") + ss.str());
    ss.str("");
    ss.clear();

    info.append(std::string("\n\t"));
    info.append(std::string("transformation system parameters> "));

    ss << alpha_z_;
    info.append(std::string("   alpha_z: ") + ss.str());
    ss.str("");
    ss.clear();

    ss << beta_z_;
    info.append(std::string("   beta_z: ") + ss.str());
    ss.str("");
    ss.clear();

    return info;
}

}
