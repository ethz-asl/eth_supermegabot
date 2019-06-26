/*
 * ContinuousTimeTransferFunction.hpp
 *
 *  Created on: Feb 19, 2018
 *      Author: Philipp Leemann
 */

#pragma once

#include "robot_utils/traits.hpp"
#include "robot_utils/filters/helper_functions.hpp"
#include "robot_utils/filters/DiscreteTimeTransferFunction.hpp"

#include <Eigen/Core>

namespace robot_utils {
/*!
 * This class implements a generic continuous time (CT) transfer function.
 *
 *                  b_0 + b_1 * s + b_2 * s^2 + ... + b_n * s^n
 *    G(s)    =    ---------------------------------------------
 *                  a_0 + a_1 * s + a_2 * s^2 + ... + a_n * s^n
 *
 *
 * The number of coefficients (n) in the numerator and denominator has to be equal. It is up to the user to
 * fill unused entries with 0.
 *
 * Internally the transfer function is transformed to discrete time (DT) using Tustin (or bilinear) transformation
 * (which maps stable CT poles inside the unit circle in the Z domain) using a given sample time T:
 *
 *            2      (z - 1)
 *    s ~=   --- *  --------
 *            T      (z + 1)
 *
 */
template<typename ValueType_, unsigned int Order_>
class ContinuousTimeTransferFunction : public DiscreteTimeTransferFunction<ValueType_, Order_> {
public:
    using BaseType = DiscreteTimeTransferFunction<ValueType_, Order_>;
    using CoefficientArray = typename BaseType::CoefficientArray;
    static constexpr unsigned int Size_ = BaseType::Size_;

    explicit ContinuousTimeTransferFunction() = default;

    /*!
     * Construct the transfer function with given sampling time and CT coefficients.
     * @param dt                        Sampling time T
     * @param numeratorCoefficients     The CT numerator coefficients b_0, ..., b_n
     * @param denominatorCoefficients   The CT denominator coefficients a_0, ..., a_n
     * @param y0                        Default value to initialize the inputs and outputs to
     */
    explicit ContinuousTimeTransferFunction(const double dt,
                                            const CoefficientArray& ctNumeratorCoefficients,
                                            const CoefficientArray& ctDenominatorCoefficients,
                                            const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setContinuousTimeCoefficients(dt, ctNumeratorCoefficients, ctDenominatorCoefficients, y0);
    }

    ~ContinuousTimeTransferFunction() override = default;

    /*!
     * Set the CT coefficients of the transfer function with sampling time T. DT coefficients are calculated internally.
     * @param dt                        Sampling time T
     * @param numeratorCoefficients     The CT numerator coefficients b_0, ..., b_n
     * @param denominatorCoefficients   The CT denominator coefficients a_0, ..., a_n
     * @param y0                        Default value to initialize the inputs and outputs to
     */
    void setContinuousTimeCoefficients(const double dt,
                                       const CoefficientArray& ctNumeratorCoefficients,
                                       const CoefficientArray& ctDenominatorCoefficients,
                                       const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        // based on "AN ALGORITHM FOR THE COMPUTATION OF THE TUSTIN  BILINEAR TRANSFORMATION" by D. Westreich

        // vectors containing coefficients to [z^0, z^1, ..., z^k]
        const double h = 2.0/dt;
        CoefficientArray dtNumerator{};
        CoefficientArray dtDenominator{};
        CoefficientArray R{};

        R[0] = 1;
        dtNumerator[0] = ctNumeratorCoefficients.back();
        dtDenominator[0] = ctDenominatorCoefficients.back();
        for(int i=static_cast<int>(Size_-2); i>=0; --i) {
            // multiply by (z+1) or (z-1) by shifting the values in the vectors

            for(unsigned int j=Size_-1; j>0; --j) {
                R[j] = R[j - 1] + R[j]; // R = R*z + R;
                dtNumerator[j]   = h * (dtNumerator[j - 1]   - dtNumerator[j]   /* P*z - P */) + ctNumeratorCoefficients[i]   * R[j];
                dtDenominator[j] = h * (dtDenominator[j - 1] - dtDenominator[j] /* Q*z - Q */) + ctDenominatorCoefficients[i] * R[j];
            }
            dtNumerator[0]   = h * (-dtNumerator[0])   + ctNumeratorCoefficients[i]   * R[0];
            dtDenominator[0] = h * (-dtDenominator[0]) + ctDenominatorCoefficients[i] * R[0];
        }

        for(unsigned int i=0; i<Size_; ++i) {
            dtNumerator[i] = dtNumerator[i] / dtDenominator.back();
            dtDenominator[i] = dtDenominator[i] / dtDenominator.back();
        }

        this->setCoefficients(dtNumerator, dtDenominator, y0);
    }

};

} /* namespace robot_utils */

