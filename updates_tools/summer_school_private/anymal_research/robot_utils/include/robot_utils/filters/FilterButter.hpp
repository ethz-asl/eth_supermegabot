/*!
* @file 	FilterButter.hpp
* @author 	Marco Hutter
* @date		Feb, 2012
* @version 	1.0
* @ingroup 	robotTask
*/

#pragma once

#include <Eigen/Core>
#include <stdio.h>

namespace robot_utils {


//! FilterButter
/*!
 * Approximate time delays for various low-pass Butterworth digital filters
 *     Order	Delay	Order	Delay	Order	Delay
    2	0.225/fc	7	0.715/fc	12	1.219/fc
    3	0.318/fc	8	0.816/fc	14	1.421/fc
    4	0.416/fc	9	0.917/fc	16	1.624/fc
    5	0.515/fc	10	1.017/fc	18	1.826/fc
    6	0.615/fc	11	1.118/fc	20	2.029/fc
    Note that cut-off frequency, fc, should be entered as Hertz.
    The measurement unit for the time delay is seconds.
    For example, a low-pass, forward-filtered signal with fc=4Hz and order N=4 will introduce a time delay of 0.416/(4 Hz)=0.104 s.

   Reference:
   @article{Manal2007678,
	title = "A general solution for the time delay introduced by a low-pass Butterworth digital filter: An application to musculoskeletal modeling",
	journal = "Journal of Biomechanics",
	volume = "40",
	number = "3",
	pages = "678 - 681",
	year = "2007",
	note = "",
	issn = "0021-9290",
	doi = "10.1016/j.jbiomech.2006.02.001",
	url = "http://www.sciencedirect.com/science/article/pii/S0021929006000480",
	author = "Kurt Manal and William Rose",
	keywords = "Signal",
	keywords = "Processing",
	keywords = "Smoothing",
	keywords = "Electromechanical delay",
	keywords = "Group delay"
	}


 * @ingroup 	robotTask
 */
class FilterButter
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	/*! Constructor
	 * @param 	dim 			Vector dimension
	 * @param  	order			Length of the filter
	 */
	FilterButter(int dim, int order);

	//! Destructor
	virtual ~FilterButter();

	/*! Initialize the lowpass filter
	 *
	 * @param fs 		Sampling frequency [rad/s] or [Hz]
	 * @param fc 		cut-off frequency [rad/s] or [Hz]
	 * @return 			true if successful
	 */
	bool initLowpass(double fs, double fc);

	/*! Deactivate the filter,
	 *
	 * @return
	 */
	bool deactivate();

	/*! adds the new value as measured point and return the next filter value
	 *
	 * @param actValues 	actual measurement
	 * @return 				filtered value
	 */
	Eigen::Matrix<double,Eigen::Dynamic,1> filterData(const Eigen::Matrix<double,Eigen::Dynamic,1>& actValues);


private:

	int dim_;
	int order_;

	bool isInit_;
	// true if the filter was not called so far
	bool firstCall_;

	Eigen::Matrix<double,1,Eigen::Dynamic> aCoeff_;
	Eigen::Matrix<double,1,Eigen::Dynamic> bCoeff_;

	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> InputValues_;
	Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> OutputValues_;


};

} /* namespace robot_utils */
