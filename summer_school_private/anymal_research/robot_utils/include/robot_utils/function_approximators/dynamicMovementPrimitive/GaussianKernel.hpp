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

#ifndef GAUSSIANKERNEL_H_
#define GAUSSIANKERNEL_H_

// system includes
#include <vector>
#include <string>
#include <Eigen/Eigen>
#include <iostream>
#include <fstream>
#include <assert.h>
#include <boost/foreach.hpp>

namespace dmp
{

class GaussianKernel
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*!
     *
     * @return
     */
    GaussianKernel();
    ~GaussianKernel();

    /*! Initialized the Gaussian kernel model
     *
     * @param numBasisFunctions
     * @param activation, design parameter, determines the variance/width of the kernels
     * 		  (value of activation is the point where a kernel intersects with its neighbors center)
     * @return True on success, false on failure
     */
    bool initialize(const int numBasisFunctions, const double activation, const bool exponentiallySpaced = false, const double canSysCutOff = 0.0);

    /*! Initializes the Gaussian kernel model using the parameters from the paramserver in the namespace parameter_namespace
     *
     * @param parameter_namespace
     * @return
     */
    //bool initialize(const std::string parameter_namespace);

    /*! initialized LWR model from message
     * @param model
     * @return
     */
    //bool initFromMessage(const dmp::Model& model);

    /*! Writes LWR model to message
     * @param model
     * @return
     */
   // bool writeToMessage(dmp::Model& model);

    /*!
     * @param x
     * @param y
     * @return True on success, false on failure
     */
    bool learnWeights(const Eigen::VectorXd& inputVectorX, const Eigen::VectorXd& targetVectorY);

    /*!
     *
     * @param x
     * @param y
     * @return True on success, false on failure
     */
    bool predict(const double queryX, double& predictionY, bool useModulationParameterX);

    /*!
     * @param inputX_vector
     * @param basis_function_matrix
     * @return True on success, false on failure
     */
    bool generateBasisFunctionMatrix(const Eigen::VectorXd& inputVectorX, Eigen::MatrixXd& basisFunctionMatrix);

    /*!
     * @param inputX
     * @param basis_function_vector
     * @return True on success, false on failure
     */
    bool generateBasisFunctionVector(const double &inputX, Eigen::VectorXd &basisFunctionVector);

    /*!
     * Gets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool getThetas(Eigen::VectorXd& thetas);

    /*!
     * Sets the theta vector
     * @param thetas
     * @return True on success, false on failure
     */
    bool setThetas(const Eigen::VectorXd& thetas);

    /*!
     * updates the theta vectors (thetas += deltaThetas)
     * @param deltaThetas
     * @return True on success, false on failure
     */
    bool updateThetas(const Eigen::VectorXd& deltaThetas);

    /*!
     * Calculated the linearly interpolated theta value for inputX
     * This function is used for logging and visualization
     *
     * @param inputX
     * @param interpolatedTheta
     * @return True on success, false on failure
     */
    bool getInterpolatedTheta(const double &xInput, double &interpolatedTheta);

    /*!
     * @param widths
     * @param centers
     * @return True on success, false on failure
     */
    bool getWidthsAndCenters(Eigen::VectorXd& widths, Eigen::VectorXd& centers);

    /*!
     * Gets the indices of the centers left and right to the value of inputX
     *
     * @param inputX
     * @param previousNeighborIndex
     * @param nextNeighborIndex
     * @return True on success, false on failure
     */
    bool getNeighborCenterIndices(const double &inputX, int &previousNeighborIndex, int &nextNeighborIndex);

     /*! Writes the LWR model to file
     *
     * @param directoryName (input) Directory name in which the LWR model is stored.
     * @return True on success, false on failure
     */
    //bool writeToDisc(const std::string directoryName);

    /*!
     * @param directoryName
     * @param itemId
     * @return
     */
    //bool initializeFromDisc(const std::string directoryName);

    /*! Returns a string containing relevant information of the Gaussian kernel model
     * @return
     */
    std::string getInfoString();

    /*! Gets the number of receptive fields
     * @return
     */
    bool getNumRFS(int& numBasisFunctions);

private:

    /*! Evaluates the kernel
     */
    double getKernel(const double inputX, const int centerIndex);

    /*! Indicates whether the Gaussian kernel model is initialized
     */
    bool initialized_;

    /*! Number of Gaussian basis functions used in this Gaussian kernel model
     *
     */
    int numBasisFunctions_;

	/*!
	 *  This value specifies at which value to neighboring functions will intersect
	 */
	double basisFunctionWidthBoundary_;

    /*! determines whether gaussians are exponentially spaced (true) or equally spaced (false)
     *  exponential scale is used to deal with the nonlinearity of the
     *  phase variable of the canonical system of a DMP
     */
    bool exponentiallySpaced_;

    /*! Centers of the receptive fields
     *
     */
    Eigen::VectorXd centers_;


    /*! Bandwidth used for each local model
     *
     */
    Eigen::VectorXd widths_;

    /*! Slopes of the local linear approximations
     *
     */
    Eigen::VectorXd thetas_;

    /*! Offsets of the local linear approximations. (currently not implemented)
     *
     */
    Eigen::VectorXd offsets_;
};

}

#endif /* GAUSSIANKERNEL_H_ */
