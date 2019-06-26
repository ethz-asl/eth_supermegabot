/*********************************************************************

 \author   Peter Fankhauser

 **********************************************************************/

#ifndef TIMELESS_GAUSSIAN_KERNEL_APPROXIMATOR_HPP_
#define TIMELESS_GAUSSIAN_KERNEL_APPROXIMATOR_HPP_

// system includes
#include <string>
#include <math.h>
#include <sstream>
#include <errno.h>
#include <stdio.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/ptr_container/ptr_vector.hpp>
#include <Eigen/Eigen>


// Local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/GaussianKernel.hpp"
#include "robot_utils/function_approximators/dynamicMovementPrimitive/constants.hpp"

namespace dmp
{

class TimelessGaussianKernelApproximator
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! constructor
     */
	TimelessGaussianKernelApproximator();

    /*! destructor
     */
    ~TimelessGaussianKernelApproximator();

    bool initialize(int numSystems, const Eigen::VectorXd num_rfs, const double activation, const double duration, const double sampling_frequency);

    bool isInitialized() const;

    bool isSetup() const;

    bool learnFromThetas(const std::vector<Eigen::VectorXd>& thetas);

    std::string getInfoString();

    /*!
     * Gets the parameters of each system
     * @param thetas
     * @return
     */
    bool getThetas(std::vector<Eigen::VectorXd>& thetas);

    /*!
     * Sets the parameters of each transformation system
     * @param thetas
     * @return
     */
    bool setThetas(const std::vector<Eigen::VectorXd>& thetas);

    /*!
     * Gets the widths and centers of each system
     * @param widths, centers
     * @return
     */
    bool getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers);

    /*!
     * @param widths
     * @param centers
     * @return
     */
    bool getWidthsAndCenters(const int system_index, Eigen::VectorXd& widths, Eigen::VectorXd& centers);

    /*!
     * Gets the number of receptive field used by the system with id system_index
     * @param trans_id
     * @param num_rfs
     * @return
     */
    bool getNumRFS(const int system_index, int& num_rfs);

    /*!
     * Gets the number of receptive fields for each transformation system
     * @param num_rfs
     * @return
     */
    bool getNumRFS(std::vector<int>& num_rfs);

    /*!
     *
     * @param num_time_steps
     * @param basis_functions
     * @return
     */
    bool getBasisFunctions(const int num_time_steps, std::vector<Eigen::MatrixXd>& basis_functions);

    /*!
     *
     * @param x_input
     * @param currentBasisFunctionsValues
     * @return
     */
    bool getBasisFunctionsValues(const double x_input, std::vector<Eigen::VectorXd> &currentBasisFunctionsValues);

    /*!
     * Update the system by setting the progress
     * @param progress
     * @return true if successful
     */
    bool update(const double progress);

    /*!
     * Returns the current progress based on the time and the duration
     * Start value is 0 and final value is 1
     * @return progress
     */
    double getProgress() const;

    bool getCurrentPosition(Eigen::VectorXd& current_position);

    int getNumSystems() const;

    bool reset();

    /*!
     * @return
     */
    std::string getClassName();


private:

    bool integrate(const double dt_total, const int num_iteration);

    void initialize();

    bool initialized_;
    bool isLearned_;
    double progress_;
    int numSystems_;

    std::vector<Eigen::VectorXd> coordinates_;
	boost::ptr_vector<GaussianKernel> gaussianKernelModels_;
};

// inline functions follow
inline bool TimelessGaussianKernelApproximator::isInitialized() const
{
    return initialized_;
}

inline int TimelessGaussianKernelApproximator::getNumSystems() const
{
    return numSystems_;
}

inline std::string TimelessGaussianKernelApproximator::getClassName()
{
    return "TimelessGaussianKernelApproximator";
}

}

#endif /* TIMELESS_GAUSSIAN_KERNEL_APPROXIMATOR_HPP_ */
