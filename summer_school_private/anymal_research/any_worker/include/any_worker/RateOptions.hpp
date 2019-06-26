/**********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2018, Remo Diethelm
 * All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Robotic Systems Lab nor ETH Zurich
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/*!
 * @file    RateOptions.hpp
 * @author  Remo Diethelm
 * @date    January, 2018
 */

#pragma once


// std
#include <atomic>
#include <cmath>
#include <string>


namespace any_worker {


/*!
 * RateOptions class.
 */
struct RateOptions
{
    //! Name for printing.
    std::string name_;
    //! Time step in seconds.
    std::atomic<double> timeStep_;
    //! If the awake time is bigger than the time step multiplied by this factor, it counts as an warning.
    std::atomic<double> maxTimeStepFactorWarning_;
    //! If the awake time is bigger than the time step multiplied by this factor, it counts as an error.
    std::atomic<double> maxTimeStepFactorError_;
    //! Boolean indicating whether the rate should be enforced.
    std::atomic<bool> enforceRate_;
    //! Linux clock ID.
    std::atomic<clockid_t> clockId_;

    /*!
     * Constructor.
     * Starts the clock. Call reset() to restart it if you do not intend to call sleep() immediately.
     * @param name                     Name for printing.
     * @param timeStep                 Time step in seconds.
     * @param maxTimeStepFactorWarning Max time step factor for warnings.
     * @param maxTimeStepFactorError   Max time step factor for errors.
     * @param enforceRate              Enforce the rate.
     * @param clockId                  Linux clock ID.
     */
    RateOptions(const std::string& name = "",
                const double timeStep = 0.0,
                const double maxTimeStepFactorWarning = 1.0,
                const double maxTimeStepFactorError = 10.0,
                const bool enforceRate = true,
                const clockid_t clockId = CLOCK_MONOTONIC)
    :   name_(name),
        timeStep_(timeStep),
        maxTimeStepFactorWarning_(maxTimeStepFactorWarning),
        maxTimeStepFactorError_(maxTimeStepFactorError),
        enforceRate_(enforceRate),
        clockId_(clockId) {}

    /*!
     * Copy constructor.
     * @param other Rate options to copy from.
     */
    RateOptions(const RateOptions& other) {
        *this = other;
    }

    /*!
     * Move constructor.
     * @param other Rate options to move.
     */
    RateOptions(RateOptions&& other)
    :   name_(std::move(other.name_)),
        timeStep_(other.timeStep_.load()),
        maxTimeStepFactorWarning_(other.maxTimeStepFactorWarning_.load()),
        maxTimeStepFactorError_(other.maxTimeStepFactorError_.load()),
        enforceRate_(other.enforceRate_.load()),
        clockId_(other.clockId_.load()) {}

    /*!
     * Assignment operator.
     * @param other Rate options.
     */
    RateOptions& operator=(const RateOptions& other) {
        name_ = other.name_;
        timeStep_ = other.timeStep_.load();
        maxTimeStepFactorWarning_ = other.maxTimeStepFactorWarning_.load();
        maxTimeStepFactorError_ = other.maxTimeStepFactorError_.load();
        enforceRate_ = other.enforceRate_.load();
        clockId_ = other.clockId_.load();
        return *this;
    }

    /*!
     * Destructor.
     */
    virtual ~RateOptions() {}

    /*!
     * Check if the rate options are valid.
     * @return True if rate options are valid.
     */
    virtual bool isValid() const {
        return (
            timeStep_ >= 0.0 &&
            !std::isinf(timeStep_) &&
            !std::isnan(timeStep_) &&
            maxTimeStepFactorWarning_ >= 0.0 &&
            !std::isnan(maxTimeStepFactorWarning_) &&
            maxTimeStepFactorError_ >= 0.0 &&
            !std::isnan(maxTimeStepFactorError_));
    }
};


} // namespace any_worker
