/*
 * DiscreteTimeTransferFunction.hpp
 *
 *  Created on: Feb 19, 2018
 *      Author: Philipp Leemann
 */

#pragma once

#include "robot_utils/traits.hpp"
#include "robot_utils/filters/helper_functions.hpp"

#include "message_logger/message_logger.hpp"

#include <Eigen/Core>

namespace robot_utils {
/*!
 * This class implements a generic discrete time (DT) transfer function.
 *
 *                  b_0 + b_1 * z + b_2 * z^2 + ... + b_n * z^n
 *    G(z)    =    ---------------------------------------------
 *                  a_0 + a_1 * z + a_2 * z^2 + ... + a_n * z^n
 *
 *
 * The number of coefficients (n) in the numerator and denominator has to be equal. It is up to the user to
 * fill unused entries with 0.
 *
 */
template<typename ValueType_, unsigned int Order_>
class DiscreteTimeTransferFunction {
public:
    static constexpr unsigned int Size_ = Order_+1;
    // todo: get underlying floating/integer type of eigen matrices?
    using CoefficientArray = std::array<double, Size_>;

    explicit DiscreteTimeTransferFunction() = default;

    /*!
     * @param numeratorCoefficients     The numerator coefficients b_0, ..., b_n
     * @param denominatorCoefficients   The denominator coefficients a_0, ..., a_n
     * @param y0                        Default value to initialize the inputs and outputs to
     */
    explicit DiscreteTimeTransferFunction(const CoefficientArray& numeratorCoefficients,
                                          const CoefficientArray& denominatorCoefficients,
                                          const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setCoefficients(numeratorCoefficients, denominatorCoefficients, y0);
    }

    virtual ~DiscreteTimeTransferFunction() = default;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(traits::is_eigen_matrix<ValueType_>::value)

    /*!
     * Set transfer function coefficients, normalize them if necessary and reset its states
     * @param numeratorCoefficients     The numerator coefficients b_0, ..., b_n
     * @param denominatorCoefficients   The denominator coefficients a_0, ..., a_n
     * @param y0                        Default value to initialize the inputs and outputs to
     */
    inline void setCoefficients(const CoefficientArray& numeratorCoefficients,
                                const CoefficientArray& denominatorCoefficients,
                                const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        std::reverse_copy(numeratorCoefficients.begin(), numeratorCoefficients.end(), numeratorCoefficients_.begin());
        std::reverse_copy(denominatorCoefficients.begin(), denominatorCoefficients.end(), denominatorCoefficients_.begin());
        if(denominatorCoefficients_[0] == 0) {
            MELO_FATAL("Coefficient a_n of DiscreteTimeTransferFunction must not be 0!");
        }
        invA_n_ = 1.0/denominatorCoefficients_[0];
        reset(y0);
    }

    /*!
     * Advance the transfer function with new data
     * @param u_k   Transfer function input
     * @return      Transfer function output
     */
    ValueType_ advance(const ValueType_& u_k)
    {
        input_[0] = u_k;

        // calculate output
        output_[0] = numeratorCoefficients_[0]*input_[0];
        for(unsigned int i=1; i<Size_; ++i) {
            output_[0] += numeratorCoefficients_[i]*input_[i] - denominatorCoefficients_[i]*output_[i];
        }
        output_[0] *= invA_n_; // dividing result by a_n here is numerically better than dividing all other coefficients by a_n

        // store previous values.
        // We could also place this code in above for loop, but assuming loop-unrolling and future vectorization
        //  -> better to have a separate loop
        for(unsigned int i=Size_-1; i>0; --i) {
            input_[i] = input_[i-1];
            output_[i] = output_[i-1];
        }

        return output_[0];
    }

    /*!
     * Reset the transfer function in- and outputs
     * @param y0    Value to reset the in- and outputs to
     */
    void reset(const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        input_.fill(y0);
        output_.fill(y0);
    }

    /*!
     * @return  Output of the transfer function
     */
    inline const ValueType_& getOutput() const { return output_[0]; }

    inline const CoefficientArray& getNumeratorCoefficients() const { return numeratorCoefficients_; }
    inline const CoefficientArray& getDenominatorCoefficients() const { return denominatorCoefficients_; }

protected:
    std::array<ValueType_, Size_> input_;
    std::array<ValueType_, Size_> output_;
    CoefficientArray numeratorCoefficients_;
    CoefficientArray denominatorCoefficients_;
    double invA_n_ = 1.0;
};

} /* namespace robot_utils */

