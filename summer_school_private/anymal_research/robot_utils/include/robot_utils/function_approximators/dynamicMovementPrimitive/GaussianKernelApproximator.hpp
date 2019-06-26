/*********************************************************************

 \author   Peter Fankhauser

 **********************************************************************/

#ifndef GAUSSIAN_KERNEL_APPROXIMATOR_HPP_
#define GAUSSIAN_KERNEL_APPROXIMATOR_HPP_

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

class GaussianKernelApproximator
{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /*! constructor
     */
	GaussianKernelApproximator();

    /*! destructor
     */
    ~GaussianKernelApproximator();

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
     * @param desired_coordinates
     * @param finished
     * @param current_position
     * @param movement_duration
     * @return
     */
    bool propagateStep(bool& movement_finished);

    /*!
     * @param desired_coordinates
     * @param movement_finished
     * @param sampling_duration
     * @param num_samples
     * @return
     */
    bool propagateStep(bool& movement_finished, const double sampling_duration, const int num_samples);

    bool getCurrentPosition(Eigen::VectorXd& current_position);

    int getNumSystems() const;

    /*!
     * Returns the current progress based on the time and the duration
     * Start value is 0 and final value is 1
     * @return progress
     */
    double getProgress() const;

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
    double time_;
    double duration_;
    double deltaT_;
    int sampleIndex_;
    int numSystems_;

    std::vector<Eigen::VectorXd> coordinates_;
	boost::ptr_vector<GaussianKernel> gaussianKernelModels_;
};

// inline functions follow
inline bool GaussianKernelApproximator::isInitialized() const
{
    return initialized_;
}

inline int GaussianKernelApproximator::getNumSystems() const
{
    return numSystems_;
}

inline std::string GaussianKernelApproximator::getClassName()
{
    return "GaussianKernelApproximator";
}

}

#endif /* GAUSSIAN_KERNEL_APPROXIMATOR_HPP_ */
