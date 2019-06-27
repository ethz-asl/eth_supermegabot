/*!
* @file    BoundedRBF1D.cpp
* @author  Christian Gehring
* @date    Feb, 2013
* @version 1.0
* @ingroup robot_utils
*/

#include "robot_utils/function_approximators/polyharmonicSplines/BoundedRBF1D.hpp"
#include <math.h>
#include <Eigen/Dense>
#include <iostream>
namespace rbf {

BoundedRBF1D::BoundedRBF1D():RBF1D()
{

}

BoundedRBF1D::~BoundedRBF1D()
{

}

void BoundedRBF1D::setRBFData(const std::vector<double>& x_input, const std::vector<double>& f_input)
{

	Eigen::Map<const Eigen::VectorXd> xInputEigen(&x_input[0], x_input.size());
	Eigen::Map<const Eigen::VectorXd> fInputEigen(&f_input[0], f_input.size());
	setRBFData(xInputEigen, fInputEigen);
}

void BoundedRBF1D::setRBFData(const Eigen::VectorXd& x_input, const Eigen::VectorXd& f_input)
{
	assert(x_input.size() == f_input.size());
	//we want: f(x_input) = f_input for all input points
	//set up the system that says that: A * w = f_input

	// additional constraints
	const int nAdditionalConstraints = 4;
	int n = (int)x_input.size();


	// insert two points at the begin and the end to force the first derivative of the
	// start and end point to be zero, such that
	// x_0 = x0 		f(x_0) = f(x_0)
	// x_1 = x0 + dx   	f(x_1) = f(x_0)
	// x_2 = x0 + 2*dx	f(x_2) = f(x_0)
	// x_n+1 = x_n-1 		f(x_n+1) = f(x_n-1)
	// x_n = x_n-1 - dx 	f(x_n) = f(x_n-1)
	// x_n-1 = x_n-1 - 2*dx f(x_n-1) = f(x_n-1)

	const double dx = 0.0001;
	xInput.resize(n+4);
	fInput.resize(n+4);
	xInput.segment(2, n) = x_input;
	fInput.segment(2, n) = f_input;
	xInput(0) = x_input(0);
	xInput(1) = x_input(0)+dx;
	xInput(2) = x_input(0)+2*dx;
	xInput(n-1+2) = x_input(n-1)-2*dx;
	xInput(n+2) = x_input(n-1)-dx;
	xInput(n+1+2) = x_input(n-1);
	fInput(0) = f_input(0);
	fInput(1) = f_input(0);
	fInput(n+2) = f_input(n-1);
	fInput(n+1+2) = f_input(n-1);
	n = (int)xInput.size();

//	std::cout << xInput << std::endl;
//	std::cout << fInput << std::endl;

	w.resize(xInput.size() + nAdditionalConstraints);

	Eigen::MatrixXd A((int)xInput.size() + nAdditionalConstraints,(int)xInput.size() + nAdditionalConstraints);
	A.setZero();


	Eigen::VectorXd b(xInput.size() + nAdditionalConstraints);
	b.setZero();

	for (int i=0;i<n;i++)
		b[i] = fInput[i];

//initialize the A values - basis function contributions
	for (int i=0;i<n;i++)
	    for (int j=0;j<n;j++)
			A(i,j) = evaluateBasisFunction(xInput[i], xInput[j]);

// in addition we want the sum of the lambdas (w) to be 0
	for (int i=0;i<n;i++){
		A(i,n) = 1;
		A(i,n+1) = xInput[i];
		A(i,n+2) = xInput[i]*xInput[i];
		A(i,n+3) = xInput[i]*xInput[i]*xInput[i];
		A(n,i) =  1;
		A(n+1,i) = xInput[i];
		A(n+2,i) = xInput[i]*xInput[i];
		A(n+2,i) = xInput[i]*xInput[i]*xInput[i];
	}


	//now solve the system...
	w = A.colPivHouseholderQr().solve(b);

	isInitialized = true;
}

double BoundedRBF1D::evaluate(double x) const {
	assert(isInitialized==true);

	int n = (int)xInput.size();


    double result = 0;

	if (n <= 0) return result;


    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * evaluateBasisFunction(xInput[k], x);
    // and now add the polynomial term contribution
    result += 1*w[n] + x*w[n+1] + x*x*w[n+2] + x*x*x*w[n+3];

	return result;
}

double BoundedRBF1D::evaluateFirstDerivative(double x) const {
	assert(isInitialized==true);
	int n = (int)xInput.size();


    double result = 0;


	if (n <= 0) return result;


    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * dBFdx(xInput[k], x);
    // and now add the polynomial term contribution
    result += w[n+1] + 2*x*w[n+2]+ 3*x*x*w[n+3];

	return result;
}

double BoundedRBF1D::evaluateSecondDerivative(double x) const {
	assert(isInitialized==true);
	int n = (int)xInput.size();
    double result = 0;


	if (n <= 0) return result;

    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * d2BFdx2(xInput[k], x);
    // and now add the polynomial term contribution
    result += 2*w[n+2]+ 6*x*w[n+3];

	return result;
}

double BoundedRBF1D::evaluateThirdDerivative(double x) const {
	assert(isInitialized==true);
	int n = (int)xInput.size();
    double result = 0;


	if (n <= 0) return result;

    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * d3BFdx3(xInput[k], x);
    // and now add the polynomial term contribution
    result += 6*w[n+3];


	return result;
}








}  /* namespace rbf */


