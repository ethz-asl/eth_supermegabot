/*********************************************************************
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 \author  Peter Pastor, Peter Fankhauser

 **********************************************************************/

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/GaussianKernel.hpp"

// import most common Eigen types
using namespace Eigen;
//USING_PART_OF_NAMESPACE_EIGEN

namespace dmp
{

GaussianKernel::GaussianKernel() :
	initialized_(false), numBasisFunctions_(0)
{
}

GaussianKernel::~GaussianKernel()
{
}

bool GaussianKernel::initialize(const int numBasisFunctions, const double activation, const bool exponentiallySpaced, const double canSysCutOff)
{
    if(initialized_) printf("Gaussian kernel model already initialized. Re-initializing with new parameters.\n");

    if (numBasisFunctions <= 0)
    {
        printf("Number of receptive fields (%i) is invalid.\n", numBasisFunctions);
        initialized_ = false;
        return initialized_;
    }
    numBasisFunctions_ = numBasisFunctions;
    centers_ = VectorXd::Zero(numBasisFunctions_);
    thetas_ = VectorXd::Zero(numBasisFunctions_);
    widths_ = VectorXd::Zero(numBasisFunctions_);
    offsets_ = VectorXd::Zero(numBasisFunctions_);

    if(exponentiallySpaced)
    {
        double last_input_x = 1.0;
        double alpha_x = -log(canSysCutOff);
        for (int i = 0; i < numBasisFunctions_; ++i)
        {
            double t = (i+1) * (1. / static_cast<double> (numBasisFunctions_ - 1)) * 1.0; // 1.0 is the default duration
            double input_x = exp(-alpha_x * t);

            // widths_(i) = -2. * log(activation) / pow(input_x - last_input_x, 2);
            widths_(i) = pow(input_x - last_input_x, 2) / -log(activation);

            centers_(i) = last_input_x;
            last_input_x = input_x;
        }
    }
    else
    {
        double diff;
        if (numBasisFunctions_ == 1)
        {
            centers_(0) = 0.5;
            diff = 0.5;
        }
        else
        {
            for (int i = 0; i < numBasisFunctions_; i++)
            {
                centers_(i) = static_cast<double> (i) / static_cast<double> (numBasisFunctions - 1);
            }
            diff = static_cast<double> (1.0) / static_cast<double> (numBasisFunctions - 1);
        }
        double width = -pow(diff / static_cast<double> (2.0), 2) / log(activation);
        for (int i = 0; i < numBasisFunctions_; i++)
        {
            widths_(i) = width;
        }
    }

    initialized_ = true;
    return initialized_;
}

bool GaussianKernel::predict(const double queryX, double &predictionY, bool useModulationParameterX)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model not initialized.\n");
        return initialized_;
    }

    double sx = 0;
    double sxtd = 0;
    for (int i = 0; i < numBasisFunctions_; i++)
    {
        double psi = getKernel(queryX, i);

        if (useModulationParameterX)
        	sxtd += psi * thetas_(i) * queryX;
        else
        	sxtd += psi * thetas_(i);

        sx += psi;
    }

    predictionY = sxtd / sx;
    return true;
}

bool GaussianKernel::generateBasisFunctionMatrix(const VectorXd &inputVectorX, MatrixXd &basisFunctionMatrix)
{
    if (inputVectorX.size() == 0)
    {
        printf("Cannot compute psi for an empty vector.\n");
        return false;
    }
    assert(basisFunctionMatrix.rows() == inputVectorX.size());
    assert(basisFunctionMatrix.cols() == centers_.size());
    for (int i = 0; i < inputVectorX.size(); i++)
    {
        for (int j = 0; j < centers_.size(); j++)
        {
            basisFunctionMatrix(i, j) = getKernel(inputVectorX[i], j);
        }
    }
    return true;
}

bool GaussianKernel::generateBasisFunctionVector(const double &inputX, VectorXd &basisFunctionVector)
{
    if(basisFunctionVector.size() != centers_.size())
    {
        printf("Size of basisFunctionVector (%i) is not equal the number of elements (%i).\n", (int) basisFunctionVector.size(), (int) centers_.size());
        return false;
    }

	for (int i = 0; i < centers_.size(); i++)
	{
		basisFunctionVector(i) = getKernel(inputX, i);
	}

    return true;
}

bool GaussianKernel::getThetas(VectorXd &thetas)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model not initialized.\n");
        return initialized_;
    }

    assert(thetas.cols() == thetas_.cols());
    assert(thetas.rows() == thetas_.rows());

    thetas = thetas_;
    return true;
}

bool GaussianKernel::setThetas(const VectorXd &thetas)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model not initialized.\n");
        return initialized_;
    }
    assert(thetas.cols() == thetas_.cols());
    assert(thetas.rows() == thetas_.rows());
    thetas_ = thetas;

    return true;
}

bool GaussianKernel::updateThetas(const VectorXd &deltaThetas)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model not initialized.\n");
        return initialized_;
    }
    assert(deltaThetas.cols() == thetas_.cols());
    assert(deltaThetas.rows() == thetas_.rows());
    thetas_ += deltaThetas;
    return true;
}

bool GaussianKernel::getWidthsAndCenters(VectorXd &widths, VectorXd &centers)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model not initialized.\n");
        return initialized_;
    }
    widths = widths_;
    centers = centers_;
    return true;
}

bool GaussianKernel::getNeighborCenterIndices(const double &inputX, int &previousNeighborIndex, int &nextNeighborIndex)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model not initialized.\n");
        return initialized_;
    }

    for (int i = 0; i < numBasisFunctions_; i++)
    {
    	if(centers_(i) < inputX)
    	{
    		if (i == 0)
    		{
    			return false;
    		}

    		previousNeighborIndex = i-1;
    		nextNeighborIndex = i;
    		return true;
    	}
    }
    return false;
}

double GaussianKernel::getKernel(const double inputX, const int centerIndex)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model not initialized.\n");
        return initialized_;
    }

    return exp(-(static_cast<double> (1.0) / widths_(centerIndex)) * pow(inputX - centers_(centerIndex), 2));
}

bool GaussianKernel::learnWeights(const VectorXd &inputVectorX, const VectorXd &targetVectorY)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model is not initialized.\n");
        return initialized_;
    }

    if(inputVectorX.size() != targetVectorY.size())
    {
        printf("Input (%i) and target (%i) vector have different sizes.\n", (int) inputVectorX.size(), (int) targetVectorY.size());
        return false;
    }

    MatrixXd basisFunctionMatrix = MatrixXd::Zero(inputVectorX.size(), centers_.size());
    if (!generateBasisFunctionMatrix(inputVectorX, basisFunctionMatrix))
    {
        printf("Could not generate basis function matrix.\n");
        return false;
    }

    MatrixXd tmp_matrix_a = MatrixXd::Zero(inputVectorX.size(), numBasisFunctions_);
    tmp_matrix_a = inputVectorX.array().square() * MatrixXd::Ones(1, numBasisFunctions_).array();  // TODO: Check
    tmp_matrix_a = tmp_matrix_a.array() * basisFunctionMatrix.array(); // TODO: Check

    VectorXd tmp_matrix_sx = VectorXd::Zero(numBasisFunctions_, 1);
    tmp_matrix_sx = tmp_matrix_a.colwise().sum();

    MatrixXd tmp_matrix_b = MatrixXd::Zero(inputVectorX.size(), numBasisFunctions_);
    tmp_matrix_b = inputVectorX.array() * targetVectorY.array() * MatrixXd::Ones(1, numBasisFunctions_).array(); // TODO: Check
    tmp_matrix_b = tmp_matrix_b.array() * basisFunctionMatrix.array(); // TODO: Check

    VectorXd tmp_matrix_sxtd = VectorXd::Zero(numBasisFunctions_, 1);
    tmp_matrix_sxtd = tmp_matrix_b.colwise().sum();

    double ridge_regression = 0.0000000001;
    thetas_ = tmp_matrix_sxtd.array() / (tmp_matrix_sx.array() + ridge_regression); // TODO: Check

    return true;
}

std::string GaussianKernel::getInfoString()
{
    std::string info;
    info.clear();

    // TODO: use utility function to convert from int to string

    std::stringstream ss;
    ss.clear();

    int precision = 4;
    ss.precision(precision);

    info.append(std::string("\t"));
    if (initialized_)
    {
        info.append(std::string("initialized: true "));
    }
    else
    {
        info.append(std::string("initialized: false"));
    }
    info.append(std::string("   numBasisFunctions: "));
    ss << numBasisFunctions_;
    info.append(ss.str());

    ss.str("");
    ss.clear();
    info.append(std::string("\n\twidths: "));
    for (int i = 0; i < widths_.size(); i++)
    {
        ss << widths_[i];
        info.append(std::string("\t") + ss.str() + std::string(" "));
        ss.str("");
        ss.clear();
    }

    ss.str("");
    ss.clear();
    info.append(std::string("\n\tthetas: "));
    for (int i = 0; i < thetas_.size(); i++)
    {
        ss << thetas_[i];
        info.append(std::string("\t") + ss.str() + std::string(" "));
        ss.str("");
        ss.clear();
    }

    ss.str("");
    ss.clear();
    info.append(std::string("\n\tcenters:"));
    for (int i = 0; i < centers_.size(); i++)
    {
        ss << centers_[i];
        info.append(std::string("\t") + ss.str() + std::string(" "));
        ss.str("");
        ss.clear();
    }

    return info;
}

bool GaussianKernel::getNumRFS(int &numBasisFunctions)
{
    if (!initialized_)
    {
        printf("Gaussian kernel is not initialized, not returning number of receptive fields.\n");
        return false;
    }
    numBasisFunctions = numBasisFunctions_;
    return true;
}

}
