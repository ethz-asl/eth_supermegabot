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

#ifndef DMP_PARAMETERS_H_
#define DMP_PARAMETERS_H_

// system includes
#include <string>
#include <stdio.h>
#include <sstream>

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/constants.hpp"

namespace dmp
{

class DmpParameters
{

public:

    friend class DiscreteMovementPrimitive;

    /*! Constructor
     */
    DmpParameters();

    /*! Destructor
     */
    ~DmpParameters();

    /*!
     * @return
     */
    bool initialize(const double sampling_frequency, const double teaching_duration, const double execution_duration, const double can_sys_cutoff, const double alpha_z, const double beta_z);

    /*!
     */
    void print();

    /*!
     */
    std::string getInfoString();

private:

    /*! Indicates whether the DMP has been learned, i.e. the parameters of the DMP has been computed/set
     */
    bool is_learned_;

    /*! Indicates whether the DMP is setup, i.e. the start, goal, and duration has been set
     *
     */
    bool is_setup_;

    /*! Indicates whether the start of the DMP has been set. This flag is used to update the start position of
     *  successive DMPs, when the start position cannot be determined beforehand.
     */
    bool is_start_set_;

    /*! Number of transformation systems (dimensions) used in the DMP
     */
    int num_transformation_systems_;

    /*! Timing parameters used during learning and integrating the system.
     *  Tau is the length of the trajectory
     */
    double tau_;
    double initial_tau_;

    /*! Time step of the integration
     */
    double delta_t_;
    double initial_delta_t_;

    /*!
     */
    std::string library_directory_name_;

    /*! Default durations are important to find values for alpha_z to obtain desired behavior
     */
    double teaching_duration_;
    double execution_duration_;

    /*! Remaining value of canonical system at end time (time = tau)
     */
    double can_sys_cutoff_;

    /*! time constant of the canonical system (defined by can_sys_cutoff_)
     */
    double alpha_x_;


    /*! time constant of the transformation systems
     */
    double alpha_z_;
    double beta_z_;

    //! Number of the current sample
    int sample_index_;

    //! Internal sampling frequency of the DMP
	int default_sampling_frequency_;
};

}

#endif /* DMP_PARAMETERS_H_ */
