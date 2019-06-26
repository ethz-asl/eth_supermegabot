/*!
 * @file 	FilterMovAvg.hpp
 * @author 	Philipp Leemann
 * @date 	Oct, 2017
 * @version 1.0
 * @ingroup robot_utils
 */

#pragma once

#include "robot_utils/traits.hpp"
#include "robot_utils/typedefs.hpp"
#include "robot_utils/filters/helper_functions.hpp"

/*!
 * Simple Moving Average Filter (FIR)
 * Computes the unweighted mean of the previous n samples.
 */

namespace robot_utils{

template<typename ValueType_>
class FilterMovAvg
{
public:

    FilterMovAvg():
            FilterMovAvg(1)
    {
    }

    /*!
     * @param nSamples	number of samples of the window
     */
    FilterMovAvg(const unsigned int nSamples, const ValueType_& initValue = getDefaultValue<ValueType_>())
    {
        setFilterParameters(nSamples, initValue);
    }

    virtual ~FilterMovAvg() = default;


    /*!
     * Resize the filter window and reset
     * @param nSamples  number of samples
     */
    inline void setFilterParameters(unsigned int nSamples, const ValueType_& resetValue = getDefaultValue<ValueType_>())
    {
        if(nSamples == 0) {
            nSamples = 1;
        }

        samples_.resize(nSamples);
        reset(resetValue);
    }

    /*!
     * Reset the filter
     * @param resetValue    value to reset the filter to
     */
    void reset(const ValueType_& y_0 = getDefaultValue<ValueType_>())
    {
        index_ = 0;
        sum_ = samples_.size() * y_0;
        std::fill(samples_.begin(), samples_.end(), y_0);
    }

    /*!
     * Filter new Values
     * @param u_k       new input Value
     * @return          filtered value
     */
    ValueType_ advance(const ValueType_& u_k)
    {
        if (samples_.size() == 1) {
            sum_ = u_k;
            return u_k;
        }

        index_ = (index_ + 1) % samples_.size();
        sum_ += u_k - samples_[index_];
        samples_[index_] = u_k;

        return getFilteredValue();
    }

    /*!
     * @return  last calculated output of the filter
     */
    inline ValueType_ getFilteredValue() const
    {
        return (sum_ / static_cast<double>(samples_.size()));
    }

private:
    VectorType<ValueType_> samples_;

    unsigned int index_;

    ValueType_ sum_;
};

}
