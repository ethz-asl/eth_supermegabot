/*
 * Delay.hpp
 *
 *  Created on: Dez. 2017
 *      Author: Fabian Jenelten
 */

#pragma once

#include "robot_utils/traits.hpp"
#include "robot_utils/typedefs.hpp"
#include "robot_utils/filters/helper_functions.hpp"

#include "message_logger/message_logger.hpp"

#include <Eigen/Core>
#include <Eigen/StdVector>

namespace robot_utils {


template<typename ValueType_>
class Delay {
public:

  Delay():
    Delay(0.0025, 0.01)
    {
    }


  Delay(const double dt, const double delay, const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setFilterParameters(dt, delay, y0);
    }

    virtual ~Delay()
    {
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(traits::is_eigen_matrix<ValueType_>::value)

    void setFilterParameters(const double dt, const double delay, const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
      dt_    = dt;
      delay_ = delay;
      state_.clear();
      if (delay_ > 0.0) {
        state_.resize(static_cast<int>(delay_/dt_));
      } else {
        state_.resize(1u);
      }
      reset(y0);
    }


    inline void setSamplingTime(const double dt, const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setFilterParameters(dt, delay_, y0);
    }

    inline void setDelay(const double delay, const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
        setFilterParameters(dt_, delay, y0);
    }

    ValueType_ advance(const ValueType_& u_k)
    {
      if (delay_ > 0.0) {
        state_.erase(state_.begin());
        state_.push_back(u_k);
      } else {
        state_.front() = u_k;
      }
      return state_.front();
    }

    void reset(const ValueType_& y0 = getDefaultValue<ValueType_>())
    {
      if (!state_.empty()) {
        std::fill(state_.begin(), state_.end(), y0);
      }
    }

    inline const ValueType_& getFilteredValue() const { return state_.front(); }

protected:
    //std::vector<ValueType_> state_;
    std::vector<ValueType_,Eigen::aligned_allocator<ValueType_>> state_;
    double delay_;
    double dt_;


};

} /* namespace robot_utils */

