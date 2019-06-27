/*!
* @file    PeriodicRBF1DC3.hpp
* @author  Christian Gehring, Stelian Coros
* @date    Feb, 2013
* @version 1.0
* @ingroup robot_utils
*/
#pragma once
#include <Eigen/Core>
#include <vector>
#include "robot_utils/function_approximators/polyharmonicSplines/RBF1D.hpp"

namespace rbf {

//! Function approximation of a periodic function with radial basis functions (polyharmonic splines) with C3 continuity
/*!
 * Compute function f(x), so that f(x_i) = f_i. The form of f is:
 * f = sum_i w_i * F_i(r) + a + b*x + c*x*x + d*x*x*x, where F_i are basis functions, one for each (x_i, f_i) pair.
 * The weights w_i, as well as the polynomial coefficients a, b, c and d are computed so that the function interpolates the input data,
 * whereby f(x_0)=f(x_n-1), f'(x_0)=f'(x_n-1),  f''(x_0)=f''(x_n-1), and f'''(x_0)=f'''(x_n-1), so that f(x) has C3 continuity
 * The basis function is F_i(r_i) = r_i*r_i*log(r_i) with r_i = (x_i-x)*(x_i-x)
 *
 */
class PeriodicRBF1DC3: public RBF1D {
public:
	//! Constructor
	PeriodicRBF1DC3();

	//! Destructor
	virtual ~PeriodicRBF1DC3();

	 /*! Compute function f(x), so that f(x_i) = f_i. The form of f is:
	 *	f = sum_i w_i * F_i(r) + a + b*x + c*x*x + d*x*x*x, where F_i are basis functions, one for each (x_i, f_i) pair
	 * 	the weights w_i, as well as the polynomial coefficients a, b, c and d are computed so that the function interpolates the input data
	 * 	The basis function is F_i(r_i) = r_i*r_i*log(r_i) with r_i = (x_i-x)*(x_i-x)
	 * @param x_input	the centers x_i
	 * @param f_input	the function values f_i
	 */
	virtual void setRBFData(const std::vector<double>& x_input, const std::vector<double>& f_input);
	virtual void setRBFData(const Eigen::VectorXd& x_input, const Eigen::VectorXd& f_input);

	/*! Evaluates the interpolation function at x, i.e. computes f(x)
	 * If x < min(x_input) or x > max(x_input), x is wrapped to the range [min(x_input) max(x_input)).
	 * @param x	value
	 * @return	f(x)
	 */
	virtual double evaluate(double x) const;

	/*! Evaluates the first derivative of the interpolation function at x, i.e. computes dfdx(x)
	 * If x < min(x_input) or x > max(x_input), x is wrapped to the range [min(x_input) max(x_input)).
	 * @param x	value
	 * @return	 dfdx(x)
	 */
	virtual double evaluateFirstDerivative(double x) const;

	/*! Evaluates the second derivative of the interpolation function at x, i.e. computes d2fdx2(x)
	 * If x < min(x_input) or x > max(x_input), x is wrapped to the range [min(x_input) max(x_input)).
	 * @param x	value
	 * @return	 d2fdx2(x)
	 */
	virtual double evaluateSecondDerivative(double x) const;

	/*! Evaluates the third derivative of the interpolation function at x, i.e. computes d3fdx3(x)
	 * If x < min(x_input) or x > max(x_input), x is wrapped to the range [min(x_input) max(x_input)).
	 * @param x	value
	 * @return	 d3fdx3(x)
	 */
	virtual double evaluateThirdDerivative(double x) const;


};


} /* namespace rbf */
