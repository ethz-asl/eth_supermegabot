/*!
* @file 	FctLinChirp.hpp
* @author 	Christian Gehring
* @date 	Jun 7, 2012
* @version 	1.0
* @ingroup 	robot_utils
* @brief
*/

#ifndef FCTLINCHIRP_HPP_
#define FCTLINCHIRP_HPP_

#include "robot_utils/function_generators/FunctionGeneratorBase.hpp"


namespace robot_utils {


//! Linear chirp function
/*!
 * @ingroup robot_utils
 */
class FctLinChirp : public FunctionGeneratorBase {
  public:
    FctLinChirp();
    virtual ~FctLinChirp();

    /*! Up and Down Sweep Function
     * @param time time [s]
     * @return	value
     */
    virtual double getUpAndDownSweepValue(double time);

    /*! Up Sweep Function
     * @param time time [s]
     * @return	value
     */
    virtual double getUpSweepValue(double time);

};

} /* namespace robot_utils */

#endif /* FCTLINCHIRP_HPP_ */
