/*
 * FirstOrderDerivative.hpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Philipp Leemann
 */

#pragma once

#include "robot_utils/filters/helper_functions.hpp"
#include "robot_utils/filters/ContinuousTimeTransferFunction.hpp"

#include "message_logger/message_logger.hpp"

namespace robot_utils {

//! First order discrete time derivative filter implementation.
/*!
 * This class implements the discrete time (DT) realization of a first order derivative continuous time (CT) transfer function. In CT it is:
 *                     mu_0*s
 *    G(s)    =    -------------
 *                   1 + tau*s
 */
template<typename ValueType_>
class FirstOrderDerivative : public ContinuousTimeTransferFunction<ValueType_, 1> {
public:
    using BaseType = ContinuousTimeTransferFunction<ValueType_, 1>;

    FirstOrderDerivative():
            FirstOrderDerivative(0.01, 0.01, 1.0)
    {
    }

    /*!
     * @param dt                        Sample time
     * @param continuousTimeConstant    The CT time constant tau (see class description)
     * @param gain                      The CT transfer function gain mu_0 (see class description)
     * @param y0                        Default value to initialize the filter to
     */
    FirstOrderDerivative(const double dt, const double continuousTimeConstant, const double gain,
                         const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setFilterParameters(dt, continuousTimeConstant, gain, y0);
    }

    ~FirstOrderDerivative() override = default;

    /*!
     * Set filter parameters and reset the filter
     * @param dt                        Sample time
     * @param continuousTimeConstant    The CT time constant tau (see class description)
     * @param gain                      The CT transfer function gain mu_0 (see class description)
     * @param y0                        Default value to initialize the filter to
     */
    void setFilterParameters(const double dt, const double continuousTimeConstant, const double gain,
                             const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        if (continuousTimeConstant < 0.0) {
            MELO_WARN("Setting a negative time constant to continuous time first order filter.");
        }

        dt_ = dt;
        tau_ = continuousTimeConstant;
        mu0_ = gain;

        this->setContinuousTimeCoefficients(dt, {0.0, gain}, {1.0, continuousTimeConstant}, y0);
    }

    /*!
     * Set the sampling time and reset the filter
     * @param dt    Sampling time
     * @param y0    Default value to initialize the filter to
     */
    inline void setSamplingTime(const double dt, const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setFilterParameters(dt, tau_, mu0_, y0);
    }

    /*!
     * Set filter continuous time constant and reset the filter
     * @param continuousTimeConstant    The CT time constant tau (see class description)
     * @param y0                        Default value to initialize the filter to
     */
    inline void setContinuousTimeConstant(const double continuousTimeConstant, const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setFilterParameters(dt_, continuousTimeConstant, mu0_, y0);
    }

    /*!
     * Set filter gain and reset the filter
     * @param gain      The CT transfer function gain mu_0 (see class description)
     * @param y0        Default value to initialize the filter to
     */
    inline void setGain(const double gain, const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setFilterParameters(dt_, tau_, gain, y0);
    }

    /*!
     * @return  Output of the filter (filtered value)
     */
    inline const ValueType_& getFilteredValue() const { return this->getOutput(); }


protected:
    //! Sample time
    double dt_;

    //! The time constant tau of the CT system.
    double tau_;

    //! The steady state gain of the CT system.
    double mu0_;

};

} /* namespace robot_utils */

