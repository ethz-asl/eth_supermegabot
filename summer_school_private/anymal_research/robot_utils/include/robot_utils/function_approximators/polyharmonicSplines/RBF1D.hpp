/*!
* @file    RBF1D.hpp
* @author  Christian Gehring, Stelian Coros
* @date    Feb, 2013
* @version 1.0
* @ingroup robot_utils
*/
#pragma once
#include <Eigen/Core>
#include <vector>

namespace rbf {

//! Function approximation of a function with radial basis functions (polyharmonic splines) with C3 continuity
/*!
 * Compute function f(x), so that f(x_i) = f_i. The form of f is:
 * f = sum_i w_i * F_i(r) + a + b*x + c*x*x + d*x*x*x, where F_i are basis functions, one for each (x_i, f_i) pair.
 * The weights w_i, as well as the polynomial coefficients a, b, c and d are computed so that the function interpolates the input data,
 * The basis function is F_i(r_i) = r_i*r_i*log(r_i) with r_i = (x_i-x)*(x_i-x)
 *
 */
class RBF1D {
public:
	//! Constructor
	RBF1D(void);

	//! Destructor
	virtual ~RBF1D(void);

	 /*! Compute function f(x), so that f(x_i) = f_i. The form of f is:
	 * @param x_input	the centers x_i
	 * @param f_input	the function values f_i
	 */
	virtual void setRBFData(const std::vector<double>& x_input, const std::vector<double>& f_input);
	virtual void setRBFData(const Eigen::VectorXd& x_input, const Eigen::VectorXd& f_input);

	/*! Evaluates the interpolation function at x, i.e. computes f(x)
	 *
	 * @param x	value
	 * @return	f(x)
	 */
	virtual double evaluate(double x) const;

	/*! Evaluates the first derivative of the interpolation function at x, i.e. computes dfdx(x)
	 *
	 * @param x	value
	 * @return	 dfdx(x)
	 */
	virtual double evaluateFirstDerivative(double x) const;

	/*! Evaluates the second derivative of the interpolation function at x, i.e. computes d2fdx2(x)
	 *
	 * @param x	value
	 * @return	 d2fdx2(x)
	 */
	virtual double evaluateSecondDerivative(double x) const;

	/*! Evaluates the third derivative of the interpolation function at x, i.e. computes d3fdx3(x)
	 *
	 * @param x	value
	 * @return	 d3fdx3(x)
	 */
	virtual double evaluateThirdDerivative(double x) const;

  /*!
   * @return gets the number of knots
   */
	int getKnotCount() const;

	/*! Gets the knot value by index
	 *
	 * @param idx	index of the knot
	 * @return value
	 */
	virtual double getKnotValue(int idx) const;

	/*! Gets the knot position by index
	 *
	 * @param idx	index of the knot
	 * @return value
	 */
	virtual double getKnotPosition(int idx) const;

	virtual double evaluateDerivative(double x, int order) const {
	  switch (order) {
	    case 1:
	      return evaluateFirstDerivative(x);
	      break;
	    case 2:
	      return evaluateSecondDerivative(x);
	      break;
	    case 3:
	      return evaluateThirdDerivative(x);
	    default:
	      throw std::runtime_error("Derivative is not yet implemented!");
	  }
	  return 0.0;
	}

protected:
	/*! Gets the radial distance
	 * @param xCenter	the center point
	 * @param x			the input variable
	 * @return	radial
	 */
	double getR(double xCenter, double x) const;
	double drdx(double xCenter, double x) const;
	double d2rdx2(double xCenter, double x) const;
	double d3rdx3(double xCenter, double x) const;

	/*! Evaluates only the basis function
	 * 	see evaluate
	 * @param r the radial distance
	 * @return
	 */
	virtual double evaluateBasisFunction(double r) const;
	virtual double evaluateBasisFunction(double xCenter, double x) const;

	virtual double dBFdr(double r) const;
	virtual double d2BFdr2(double r) const;
	virtual double d3BFdr3(double r) const;
	double dBFdx(double xCenter, double x) const;
	double d2BFdx2(double xCenter, double x) const;
	double d3BFdx3(double xCenter, double x) const;


protected:
	//! true if RFB data was set
	bool isInitialized;

	//! points (knots) that are interpolated
	Eigen::VectorXd xInput;

	//! values of points that are interpolated
	Eigen::VectorXd fInput;

	//! stacked array of weights for the basis functions and the polynomial
	Eigen::VectorXd w;



protected:
	/*! modulu - similar to matlab's mod()
 	 * result is always positive. not similar to fmod()
 	 * Mod(-3,4)= 1   fmod(-3,4)= -3
	 * @param x	nominator
	 * @param y denominator
	 * @return	x-y*floor(x/y)
	 */
	double Mod(double x, double y) const;

	/*! Wrap x to [min, max)
	 * @param x 	input variable
	 * @param min	minimal value
	 * @param max	maximal value
	 * @return	wrapped variable
	 */
	double wrapToRange(double x, double min, double max) const;

};


} /* namespace rbf */
