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
 * @file    Rate.hpp
 * @author  Remo Diethelm
 * @date    January, 2018
 */

#pragma once


// std
#include <ctime>

// any worker
#include "any_worker/RateOptions.hpp"


namespace any_worker {


/*!
 * Rate class for repeated processing at a certain rate.
 * The processing time is defined as the "awake" time, the time between the "sleep"
 * time. This class implements a sleep() method for sleeping for the remaining time
 * of the time step. Also statistics about the awake time are taken.
 *
 * If the awake time is bigger than the time step (timing is violated),
 * there are two behaviors implemented:
 *
 * Enforce the rate: Try to keep the rate constant by compensating with shorter time steps.
 * |       |       |       |       |       |       |
 * |===--->|===--->|============|===|===-->|===--->|
 * |       |       |       |       |       |       |
 *
 * Do not enforce the rate: Try to keep the single time step constant.
 * |       |       |            |       |       |       |
 * |===--->|===--->|============|===--->|===--->|===--->|
 * |       |       |            |       |       |       |
 *
 * Legend:
 *  |  Step time
 * === Awake time
 * --> Sleep time
 *
 * The time step, the time step thresholds for warnings and errors and the setting to
 * enforce the rate can be changed during run-time of this class.
 */
class Rate
{
protected:
    //! Factor storing nanoseconds per seconds.
    static constexpr long int NSecPerSec_ = 1e9;
    //! Factor storing seconds per nanoseconds.
    static constexpr double SecPerNSec_ = 1.0/NSecPerSec_;

    //! Rate options.
    RateOptions options_;

    //! Point in time when the most recent sleep() started.
    timespec sleepStartTime_;
    //! Point in time when the most recent sleep() ended.
    timespec sleepEndTime_;
    //! Point in time when the most recent step should have started.
    //! If the timing is fine, the step time is equal to the sleep end time.
    timespec stepTime_;
    //! Counter storing how many times sleep has been called.
    unsigned int numTimeSteps_ = 0;
    //! Counter storing how many times a time step took longer than the warning threshold.
    unsigned int numWarnings_ = 0;
    //! Counter storing how many times a time step took longer than the error threshold, not considering warnings.
    unsigned int numErrors_ = 0;
    //! Point in time when the last warning message was printed.
    timespec lastWarningPrintTime_;
    //! Point in time when the last error message was printed.
    timespec lastErrorPrintTime_;
    //! Most recent time which elapsed between subsequent calls of sleep().
    double awakeTime_ = 0.0;
    //! Mean of the time which elapsed between subsequent calls of sleep().
    double awakeTimeMean_ = 0.0;
    //! Helper variable to compute the variance of the time step which elapsed between subsequent calls of sleep().
    double awakeTimeM2_ = 0.0;

public:
    /*!
     * Simple constructor.
     * Starts the clock. Call reset() to restart it if you do not intend to call sleep() immediately.
     * @param name     Name for printing.
     * @param timeStep Time step in seconds.
     */
    Rate(const std::string& name,
         const double timeStep);

    /*!
     * Advanced constructor.
     * Starts the clock. Call reset() to restart it if you do not intend to call sleep() immediately.
     * @param options Rate options.
     */
    Rate(const RateOptions& options);

    /*!
     * Move constructor.
     * @param other Rate to move.
     */
    Rate(Rate&& other);

    /*!
     * Get the rate options by reference.
     * @return Rate options by reference.
     */
    RateOptions& getOptions() { return options_; }

    /*!
     * Get the rate options by constant reference.
     * @return Rate options by constant reference.
     */
    const RateOptions& getOptions() const { return options_; }

    /*!
     * Reset the internal memory and restart the step time.
     */
    void reset();

    /*!
     * Sleep for the rest of the time step.
     */
    void sleep();

    /*!
     * Get the time when the most recent sleep() started.
     * @return Time when the most recent sleep() started.
     */
    const timespec& getSleepStartTime() const { return sleepStartTime_; }

    /*!
     * Get the time when the most recent sleep() was supposed to end.
     * @return Time when the most recent sleep() was supposed to end.
     */
    const timespec& getSleepEndTime() const { return sleepEndTime_; }

    /*!
     * Get the time when the most recent step should have started.
     * @return Time when the most recent step should have started.
     */
    const timespec& getStepTime() const { return stepTime_; }

    /*!
     * Get the number of time steps.
     * @return Number of time steps.
     */
    unsigned int getNumTimeSteps() const { return numTimeSteps_; }

    /*!
     * Get the number of warnings.
     * @return Number of warnings.
     */
    unsigned int getNumWarnings() const { return numWarnings_; }

    /*!
     * Get the number of errors, not considering warnings.
     * @return Number of errors, not considering warnings.
     */
    unsigned int getNumErrors() const { return numErrors_; }

    /*!
     * Get the most recent time which elapsed between subsequent calls of sleep().
     * @return Elapsed time in seconds.
     */
    double getAwakeTime() const;

    /*!
     * Get the mean of the time which elapsed between subsequent calls of sleep().
     * @return Mean in seconds.
     */
    double getAwakeTimeMean() const;

    /*!
     * Get the variance of the time which elapsed between subsequent calls of sleep().
     * @return Variance in seconds^2.
     */
    double getAwakeTimeVar() const;

    /*!
     * Get the standard deviation of the time which elapsed between subsequent calls of sleep().
     * @return Standard deviation in seconds.
     */
    double getAwakeTimeStdDev() const;

    /*!
     * Get the duration between start and end time points.
     * @param start Start time point.
     * @param end   End time point.
     * @return Duration in seconds.
     */
    static double GetDuration(const timespec& start, const timespec& end);

    /*!
     * Add a duration to a time point.
     * @param time     Time point.
     * @param duration Duration to add in seconds.
     */
    static void AddDuration(timespec& time, const double duration);
};


} // namespace any_worker
