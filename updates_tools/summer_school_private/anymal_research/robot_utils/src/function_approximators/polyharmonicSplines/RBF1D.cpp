/*!
* @file    RBF1D.cpp
* @author  Christian Gehring, Stelian Coros
* @date    Feb, 2013
* @version 1.0
* @ingroup robot_utils
*/

#include "robot_utils/function_approximators/polyharmonicSplines/RBF1D.hpp"
#include <math.h>
#include <Eigen/Dense>

namespace rbf {

RBF1D::RBF1D():isInitialized(false)
{

}

RBF1D::~RBF1D()
{

}

void RBF1D::setRBFData(const std::vector<double>& x_input, const std::vector<double>& f_input)
{

	Eigen::Map<const Eigen::VectorXd> xInputEigen(&x_input[0], x_input.size());
	Eigen::Map<const Eigen::VectorXd> fInputEigen(&f_input[0], f_input.size());
	setRBFData(xInputEigen, fInputEigen);
}

void RBF1D::setRBFData(const Eigen::VectorXd& x_input, const Eigen::VectorXd& f_input)
{
	assert(x_input.size() == f_input.size());

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

		A(n+1,i) = x_input[i];
		A(n+2,i) = x_input[i]*x_input[i];
	}



	//now solve the system...
	w = A.colPivHouseholderQr().solve(b);

	isInitialized = true;
}

double RBF1D::evaluate(double x) const {
	assert(isInitialized==true);

	int n = (int)xInput.size();


    double result = 0;

	if (n <= 0) return result;


    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * evaluateBasisFunction(xInput[k], x);
    // and now add the polynomial term contribution
    result += 1*w[n] + x*w[n+1] + x*x*w[n+2];

	return result;
}

double RBF1D::evaluateFirstDerivative(double x) const {
	assert(isInitialized==true);
	int n = (int)xInput.size();
    double result = 0;
	if (n <= 0) return result;


    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * dBFdx(xInput[k], x);
    // and now add the polynomial term contribution
    result += w[n+1] + 2*x*w[n+2];

	return result;
}

double RBF1D::evaluateSecondDerivative(double x) const {
	assert(isInitialized==true);
	int n = (int)xInput.size();
    double result = 0;


	if (n <= 0) return result;

    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * d2BFdx2(xInput[k], x);
    // and now add the polynomial term contribution
    result += 2*w[n+2];

	return result;
}

double RBF1D::evaluateThirdDerivative(double x) const {
	assert(isInitialized==true);
	int n = (int)xInput.size();
    double result = 0;


	if (n <= 0) return result;

    // Evaluate interpolant at x - first the contributions of the basis functions
    for (int k=0; k<n; k++)
        result += w[k] * d3BFdx3(xInput[k], x);
    // and now add the polynomial term contribution


	return result;
}


double RBF1D::getR(double xCenter, double x) const {
	return (xCenter - x) * (xCenter - x);
}

double RBF1D::drdx(double xCenter, double x) const {
	return  -2*(xCenter - x);
}



double RBF1D::d2rdx2(double xCenter, double x) const {
	return 2;
}

double RBF1D::d3rdx3(double xCenter, double x) const{
	return 0;
}

double RBF1D::evaluateBasisFunction(double r) const {
    if (r==0)
        return 0;
    else
        return r*r*log(r);
}

double RBF1D::dBFdr(double r) const {
    if (r==0)
        return 0;
    else
        return 2*r*log(r) + r;

}

double RBF1D::d2BFdr2(double r) const {
   if (r==0)
        return 0;
    else
        return 3 + 2*log(r);
}

double RBF1D::d3BFdr3(double r) const {
   if (r==0)
        return 0;
    else
        return 2.0/r;
}


double RBF1D::evaluateBasisFunction(double xCenter, double x) const {
	return evaluateBasisFunction(getR(xCenter, x));
}

double RBF1D::dBFdx(double xCenter, double x) const {
    double r = getR(xCenter, x);
    return dBFdr(r) * drdx(xCenter, x);
}

double RBF1D::d2BFdx2(double xCenter, double x) const {
    double r = getR(xCenter, x);
    return d2BFdr2(r) * drdx(xCenter, x) * drdx(xCenter, x) + dBFdr(r) * d2rdx2(xCenter, x);
}

double RBF1D::d3BFdx3(double xCenter, double x) const {
    double r = getR(xCenter, x);
    return d3BFdr3(r)*drdx(xCenter, x)*drdx(xCenter, x)*drdx(xCenter, x) + 2*d2BFdr2(r)*drdx(xCenter,x)*d2rdx2(xCenter,x) + d2BFdr2(r)*drdx(xCenter,x)*d2rdx2(xCenter, x) + dBFdr(r)*d3rdx3(xCenter,x);
}


double RBF1D::Mod(double x, double y) const {
    if (0 == y)
        return x;
    return x - y * floor(x/y);
}


double RBF1D::wrapToRange(double x, double min, double max) const {
    return Mod(x + max, max-min) + min;
}


int RBF1D::getKnotCount() const
{
    return xInput.size();
}

double RBF1D::getKnotValue(int idx) const
{
    return fInput(idx);
}
double RBF1D::getKnotPosition(int idx) const
{
    return xInput(idx);
}




}  /* namespace rbf */


