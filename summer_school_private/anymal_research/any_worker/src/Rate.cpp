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
 * @file    Rate.cpp
 * @author  Remo Diethelm
 * @date    January, 2018
 */

// message logger
#include <message_logger/message_logger.hpp>

// any worker
#include "any_worker/Rate.hpp"


namespace any_worker {


Rate::Rate(const std::string& name,
           const double timeStep)
:   Rate(RateOptions(name, timeStep)) {}

Rate::Rate(const RateOptions& options)
:   options_(options) {
    reset();
}

Rate::Rate(Rate&& other)
:   options_(std::move(other.options_)),
    sleepStartTime_(std::move(other.sleepStartTime_)),
    sleepEndTime_(std::move(other.sleepEndTime_)),
    stepTime_(std::move(other.stepTime_)),
    numTimeSteps_(std::move(other.numTimeSteps_)),
    numWarnings_(std::move(other.numWarnings_)),
    numErrors_(std::move(other.numErrors_)),
    lastWarningPrintTime_(std::move(other.lastWarningPrintTime_)),
    lastErrorPrintTime_(std::move(other.lastErrorPrintTime_)),
    awakeTime_(std::move(other.awakeTime_)),
    awakeTimeMean_(std::move(other.awakeTimeMean_)),
    awakeTimeM2_(std::move(other.awakeTimeM2_)) {
    reset();
}

void Rate::reset() {
    // Reset the counters and statistics.
    numTimeSteps_ = 0;
    numWarnings_ = 0;
    numErrors_ = 0;
    awakeTime_ = 0.0;
    awakeTimeMean_ = 0.0;
    awakeTimeM2_ = 0.0;
    lastWarningPrintTime_.tv_sec = 0;
    lastWarningPrintTime_.tv_nsec = 0;
    lastErrorPrintTime_.tv_sec = 0;
    lastErrorPrintTime_.tv_nsec = 0;

    // Update the sleep time to the current time.
    timespec now;
    clock_gettime(options_.clockId_, &now);
    sleepStartTime_ = now;
    sleepEndTime_ = now;
    stepTime_ = now;
}

void Rate::sleep() {
    // Get the current time and compute the time which the thread has been awake.
    clock_gettime(options_.clockId_, &sleepStartTime_);
    awakeTime_ = GetDuration(sleepEndTime_, sleepStartTime_);

    // Update the statistics. The algorithm is described here:
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    numTimeSteps_++;
    const double delta = awakeTime_ - awakeTimeMean_;
    awakeTimeMean_ += delta / numTimeSteps_;
    const double delta2 = awakeTime_ - awakeTimeMean_;
    awakeTimeM2_ += delta * delta2;

    if(options_.timeStep_ == 0.0) {
        sleepEndTime_ = sleepStartTime_;
    }else{
        // Check if the awake exceeds the threshold for warnings or errors.
        if (awakeTime_ > options_.maxTimeStepFactorError_ * options_.timeStep_) {
            // Count and print the error.
            numErrors_++;
            if (GetDuration(lastErrorPrintTime_, sleepStartTime_) > 1.0) {
                MELO_ERROR_STREAM(
                        "Rate '" << options_.name_ << "': " << "Processing took too long (" << awakeTime_ << " s > " << options_.timeStep_.load()
                                 << " s). " << "Number of errors: " << numErrors_ << ".");
                lastErrorPrintTime_ = sleepStartTime_;
            }
        } else if (awakeTime_ > options_.maxTimeStepFactorWarning_ * options_.timeStep_) {
            // Print and count the warning (only if no error).
            numWarnings_++;
            if (GetDuration(lastWarningPrintTime_, sleepStartTime_) > 1.0) {
                MELO_WARN_STREAM(
                        "Rate '" << options_.name_ << "': " << "Processing took too long (" << awakeTime_ << " s > " << options_.timeStep_.load()
                                 << " s). " << "Number of warnings: " << numWarnings_ << ".");
                lastWarningPrintTime_ = sleepStartTime_;
            }
        }

        // Compute the next desired step time.
        AddDuration(stepTime_, options_.timeStep_);

        // Get the current time again and check if the step time has already past.
        clock_gettime(options_.clockId_, &sleepEndTime_);
        const bool isBehind = (GetDuration(sleepEndTime_, stepTime_) < 0.0);
        if (isBehind) {
            if (!options_.enforceRate_) {
                // We are behind schedule but do not enforce the rate, so we increase the length of
                // the current time step by setting the desired step time to when sleep() ends.
                stepTime_ = sleepEndTime_;
            }
        } else {
            // sleep() will finish in time. The end of the function will be at
            // the target time of clock_nanosleep(..).
            sleepEndTime_ = stepTime_;

            // Sleep until the step time is reached.
            clock_nanosleep(options_.clockId_, TIMER_ABSTIME, &stepTime_, NULL);

            // Do nothing here to ensure sleep() does not consume time after clock_nanosleep(..).
        }
    }
}

double Rate::getAwakeTime() const {
    if (numTimeSteps_ == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    } else {
        return awakeTime_;
    }
}

double Rate::getAwakeTimeMean() const {
    if (numTimeSteps_ == 0) {
        return std::numeric_limits<double>::quiet_NaN();
    } else {
        return awakeTimeMean_;
    }
}

double Rate::getAwakeTimeVar() const {
    if (numTimeSteps_ <= 1) {
        return std::numeric_limits<double>::quiet_NaN();
    } else {
        return awakeTimeM2_ / (numTimeSteps_ - 1);
    }
}

double Rate::getAwakeTimeStdDev() const {
    return std::sqrt(getAwakeTimeVar());
}

double Rate::GetDuration(const timespec& start, const timespec& end) {
    return (end.tv_sec - start.tv_sec) +
           (end.tv_nsec - start.tv_nsec) * SecPerNSec_;
}

void Rate::AddDuration(timespec& time, const double duration) {
    time.tv_nsec += duration * NSecPerSec_;
    time.tv_sec += time.tv_nsec / NSecPerSec_;
    time.tv_nsec = time.tv_nsec % NSecPerSec_;
}


} // namespace any_worker
