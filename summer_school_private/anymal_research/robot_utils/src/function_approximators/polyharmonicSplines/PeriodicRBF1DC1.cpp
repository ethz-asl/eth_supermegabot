/*!
* @file    PeriodicRBF1DC1.cpp
* @author  Christian Gehring, Stelian Coros
* @date    Feb, 2013
* @version 1.0
* @ingroup robot_utils
*/

#include "robot_utils/function_approximators/polyharmonicSplines/PeriodicRBF1DC1.hpp"
#include <math.h>
#include <Eigen/Dense>

namespace rbf {

PeriodicRBF1DC1::PeriodicRBF1DC1():RBF1D()
{

}

PeriodicRBF1DC1::~PeriodicRBF1DC1()
{

}

void PeriodicRBF1DC1::setRBFData(const std::vector<double>& x_input, const std::vector<double>& f_input)
{

	Eigen::Map<const Eigen::VectorXd> xInputEigen(&x_input[0], x_input.size());
	Eigen::Map<const Eigen::VectorXd> fInputEigen(&f_input[0], f_input.size());
	setRBFData(xInputEigen, fInputEigen);
}

void PeriodicRBF1DC1::setRBFData(const Eigen::VectorXd& x_input, const Eigen::VectorXd& f_input)
{
	assert(x_input.size() == f_input.size());
	assert(f_input[0] == f_input[f_input.size()-1]);
	//we want: f(x_input) = f_input for all input points
	//sum w_i = 0 - somewhat arbitrary i guess - far away, only the polynomial part remains...
	//f'(x_0) = f'(x_{n-1}) - first derivative should be the same at the end points

	//set up the system that says that: A * w = f_input

	const int nAdditionalConstraints = 3;
	int n = (int)x_input.size();
	xInput = x_input;
	fInput = f_input;
	w.resize(xInput.size() + nAdditionalConstraints);

	Eigen::MatrixXd A((int)xInput.size() + nAdditionalConstraints,(int)xInput.size() + nAdditionalConstraints);
	A.setZero();


	Eigen::VectorXd b(xInput.size() + nAdditionalConstraints);
	b.setZero();

	for (int i=0;i<n;i++)
		b[i] = f_input[i];

//initialize the A values - basis function contributions
	for (int i=0;i<n;i++)
	    for (int j=0;j<n;j++)
			A(i,j) = evaluateBasisFunction(x_input[i], x_input[j]);

// in addition we want the sum of the lambdas to be 0, and the
// first and second derivatives to match at the end points x1 and x2 - also, add the polynomial contribution

	for (int i=0;i<n;i++){
		A(i,n) = 1;
		A(i,n+1) = x_input[i];
		A(i,n+2) = x_input[i]*x_input[i];
		A(n,i) =  1;
		A(n+1,i) = dBFdx(x_input[i],x_input[0]) - dBFdx(x_input[i],x_input[n-1]);


	}

	// first derivative
	 A(n+1, n+2) =  2*(x_input[0]-x_input[n-1]);




	//now solve the system...
	w = A.colPivHouseholderQr().solve(b);

	isInitialized = true;
}

double PeriodicRBF1DC1::evaluate(double x) const {
	assert(isInitialized==true);

	int n = (int)xInput.size();
	x = wrapToRange(x, xInput[0], xInput[n-1]);

    double result = 0;

	if (n <= 0) return result;


    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * evaluateBasisFunction(xInput[k], x);
    // and now add the polynomial term contribution
    result += 1*w[n] + x*w[n+1] + x*x*w[n+2];;

	return result;
}

double PeriodicRBF1DC1::evaluateFirstDerivative(double x)  const {
	assert(isInitialized==true);
	int n = (int)xInput.size();
	x = wrapToRange(x, xInput[0], xInput[n-1]);

    double result = 0;


	if (n <= 0) return result;


    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * dBFdx(xInput[k], x);
    // and now add the polynomial term contribution
    result += w[n+1] + 2*x*w[n+2];

	return result;
}

double PeriodicRBF1DC1::evaluateSecondDerivative(double x) const {
	assert(isInitialized==true);
	int n = (int)xInput.size();
	x = wrapToRange(x, xInput[0], xInput[n-1]);
    double result = 0;


	if (n <= 0) return result;

    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * d2BFdx2(xInput[k], x);
    // and now add the polynomial term contribution
    result += 2*w[n+2];

	return result;
}

double PeriodicRBF1DC1::evaluateThirdDerivative(double x) const {
	assert(isInitialized==true);
	int n = (int)xInput.size();
	x = wrapToRange(x, xInput[0], xInput[n-1]);
    double result = 0;


	if (n <= 0) return result;

    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * d3BFdx3(xInput[k], x);
    // and now add the polynomial term contribution
//    result += 0;


	return result;
}








}  /* namespace rbf */


