/*********************************************************************

 \author  Peter Fankhauser

 **********************************************************************/

// local includes
#include "robot_utils/function_approximators/dynamicMovementPrimitive/TimelessGaussianKernelApproximator.hpp"
#include "kindr/common/assert_macros.hpp"

// import most common Eigen types
using namespace Eigen;

namespace dmp
{

TimelessGaussianKernelApproximator::TimelessGaussianKernelApproximator() :
    initialized_(false), numSystems_(0)
{
}

TimelessGaussianKernelApproximator::~TimelessGaussianKernelApproximator()
{

}

bool TimelessGaussianKernelApproximator::initialize(int numSystems, const Eigen::VectorXd num_rfs, const double activation, const double duration, const double sampling_frequency)
{
    // overwrite number of system
    if (numSystems <= 0)
    {
        printf("Number of systems %i is not valid\n", numSystems);
        initialized_ = false;
        return initialized_;
    }
    numSystems_ = numSystems;

    // initialized systems using the gaussian kernel model parameters
    for (int i = 0; i < numSystems_; i++)
    {
        printf("Initializing Gaussian kernel system %i.\n",i);
        gaussianKernelModels_.push_back(new GaussianKernel());
        if (!gaussianKernelModels_[i].initialize(num_rfs(i), activation))
        {
            printf("Could not initialize Gaussian kernel system %i.\n", i);
            initialized_ = false;
            return initialized_;
        }
    }

    // coordinates
    coordinates_.resize(numSystems_, VectorXd(dmp::POS_VEL_ACC));

    initialized_ = true;
    return initialized_;
}

bool TimelessGaussianKernelApproximator::learnFromThetas(const std::vector<VectorXd>& thetas)
{
    if (!initialized_)
    {
        printf("DMP is not initialized.\n");
        isLearned_ = false;
        return isLearned_;
    }

    if (!setThetas(thetas))
    {
        printf("Could not set theta parameters.\n");
        isLearned_ = false;
        return isLearned_;
    }

	printf("Gaussian kernel model learned from Thetas.\n");
	isLearned_ = true;
    return isLearned_;
}

bool TimelessGaussianKernelApproximator::getCurrentPosition(VectorXd &current_position)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model is not initialized.\n");
        initialized_ = false;
        return initialized_;
    }

    if (current_position.size() != numSystems_)
    {
        printf("Provided vector has wrong size (%i), required size is (%i). Cannot get current position.\n", (int) current_position.size(), numSystems_);
        return false;
    }

    for (int i = 0; i < numSystems_; i++)
    {
    	current_position(i) = coordinates_.at(i)(_POS_);
    }
    return true;
}

bool TimelessGaussianKernelApproximator::getThetas(std::vector<VectorXd>& thetas)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model is not initialized.\n");
        initialized_ = false;
        return initialized_;
    }

    thetas.clear();
    for (int i = 0; i < numSystems_; i++)
    {
        int num_rfs;
        if (!gaussianKernelModels_[i].getNumRFS(num_rfs))
        {
            printf("Could not get number of receptive fields.\n");
            return false;
        }

        VectorXd theta_vector = VectorXd::Zero(num_rfs);
        if (!gaussianKernelModels_[i].getThetas(theta_vector))
        {
            printf("Could not retrieve thetas from Gaussian kernel model %i.\n",i);
            return false;
        }
        thetas.push_back(theta_vector);
    }
    return true;
}

bool TimelessGaussianKernelApproximator::setThetas(const std::vector<VectorXd>& thetas)
{
    if(static_cast<int>(thetas.size()) != numSystems_)
    {
    	printf("Number of thetas (%i) is not equal to number of systems (%i).\n", (int) thetas.size(), numSystems_);
    	return false;
    }

    for (int i = 0; i < numSystems_; i++)
    {
        if (!gaussianKernelModels_[i].setThetas(thetas[i]))
        {
            printf("Could not set thetas of system %i.\n",i);
            return false;
        }
    }
    return true;
}

bool TimelessGaussianKernelApproximator::getWidthsAndCenters(std::vector<Eigen::VectorXd>& widths, std::vector<Eigen::VectorXd>& centers)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model is not initialized.\n");
        initialized_ = false;
        return initialized_;
    }

    widths.clear();
    centers.clear();

    for (int i = 0; i < numSystems_; i++)
    {
        int num_rfs;
        if (!gaussianKernelModels_[i].getNumRFS(num_rfs))
        {
            printf("Could not get number of receptive fields.\n");
            return false;
        }

        VectorXd centers_vector = VectorXd::Zero(num_rfs);
        VectorXd widths_vector = VectorXd::Zero(num_rfs);
        if (!gaussianKernelModels_[i].getWidthsAndCenters(widths_vector, centers_vector))
        {
            printf("Could not retrieve thetas from gaussian kernel model %i.\n",i);
            return false;
        }

        widths.push_back(widths_vector);
        centers.push_back(centers_vector);
    }
    return true;
}

bool TimelessGaussianKernelApproximator::getWidthsAndCenters(const int system_index, VectorXd& widths, VectorXd& centers)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model is not initialized.\n");
        initialized_ = false;
        return initialized_;
    }

    int num_rfs;
    KINDR_ASSERT_TRUE(std::runtime_error, !getNumRFS(system_index, num_rfs), "Could not get number of RFS.\n");
    KINDR_ASSERT_TRUE(std::runtime_error, widths.size() == num_rfs, "");
    KINDR_ASSERT_TRUE(std::runtime_error, centers.size() == num_rfs, "");

    if (!gaussianKernelModels_[system_index].getWidthsAndCenters(widths, centers))
    {
        printf("Could not get widths and centers of system %i.\n", system_index);
        return false;
    }
    return true;
}

bool TimelessGaussianKernelApproximator::getBasisFunctions(const int num_time_steps, std::vector<MatrixXd>& basis_functions)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model is not initialized.\n");
        initialized_ = false;
        return initialized_;
    }

    basis_functions.clear();
    for (int i = 0; i < numSystems_; i++)
    {
        int num_rfs;
        if (!getNumRFS(i, num_rfs))
        {
            return false;
        }
        MatrixXd basis_function_matrix = MatrixXd::Zero(num_time_steps, num_rfs);
        VectorXd x_input_vector = VectorXd::Zero(num_time_steps);
        double dx = static_cast<double> (1.0) / static_cast<double> (num_time_steps - 1);
        x_input_vector(0) = 0.0;
        for (int j = 1; j < num_time_steps; j++)
        {
            x_input_vector(j) = x_input_vector(j - 1) + dx;
        }
        if (!gaussianKernelModels_[i].generateBasisFunctionMatrix(x_input_vector, basis_function_matrix))
        {
            printf("Gaussian kernel basis function generation failed!\n");
            return false;
        }
        basis_functions.push_back(basis_function_matrix);
    }
    return true;
}

bool TimelessGaussianKernelApproximator::getBasisFunctionsValues(const double x_input, std::vector<VectorXd> &basisFunctionsValues)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model is not initialized.\n");
        initialized_ = false;
        return initialized_;
    }

    if (basisFunctionsValues.size() != numSystems_)
    {
        printf("basisFunctionsValues vector size (%i) does not match the number of transformation systems %i.\n", (int) basisFunctionsValues.size(), numSystems_);
        return false;
    }

    for (int i = 0; i < numSystems_; i++)
    {
        int num_rfs;
        if (!getNumRFS(i, num_rfs))
        {
            return false;
        }

        basisFunctionsValues.at(i).setZero(num_rfs);

        if (!gaussianKernelModels_[i].generateBasisFunctionVector(x_input, basisFunctionsValues.at(i)))
        {
            printf("Gaussian kernel basis function generation failed!\n");
            return false;
        }
    }
    return true;
}

bool TimelessGaussianKernelApproximator::getNumRFS(const int system_index, int& num_rfs)
{
    if (!initialized_)
    {
        printf("Gaussian kernel model is not initialized.\n");
        initialized_ = false;
        return initialized_;
    }

    if ((system_index < 0) || (system_index >= numSystems_))
    {
        printf("Could not get number of receptive fields, the system id (%i) is invalid.\n", system_index);
        return false;
    }

    return gaussianKernelModels_[system_index].getNumRFS(num_rfs);
}

bool TimelessGaussianKernelApproximator::getNumRFS(std::vector<int>& num_rfs)
{
    num_rfs.clear();
    for (int i = 0; i < numSystems_; i++)
    {
        int tmp_num_rfs;
        if (!getNumRFS(i, tmp_num_rfs))
        {
            return false;
        }
        num_rfs.push_back(tmp_num_rfs);
    }
    return true;
}

bool TimelessGaussianKernelApproximator::update(const double progress)
{
	if (!isLearned_)
    {
        printf("Gaussian kernel model is not learned.\n");
        return false;
    }

	progress_ = progress;

    for (int i = 0; i < numSystems_; i++)
	{
		// Compute nonlinearity using Gaussian kernel model
		double prediction = 0;
		if (!gaussianKernelModels_[i].predict(progress_, prediction, false))
		{
			printf("Could not predict output.\n");
			return false;
		}

		coordinates_.at(i)(_POS_) = prediction;
		coordinates_.at(i)(_VEL_) = 0.0; // TODO
		coordinates_.at(i)(_ACC_) = 0.0; // TODO
	}

    return true;
}

double TimelessGaussianKernelApproximator::getProgress() const
{
	return progress_;
}

bool TimelessGaussianKernelApproximator::reset()
{
	for (int i = 0; i < numSystems_; i++)
	{
		coordinates_.at(i).setConstant(0.0);
	}

    return true;
}

std::string TimelessGaussianKernelApproximator::getInfoString()
{
    return std::string("");
/*    std::string info("");
    std::stringstream ss;

    //ss << item_id_;
    info.append(std::string("id: ") + ss.str());
    ss.str("");
    ss.clear();

   // info.append(std::string("\t"));
    //info.append(std::string("name: ") + item_name_);

    //info.append(std::string("\n\t"));
    //info.append(std::string("description: ") + description_);

    info.append(std::string("\n\t"));
    info.append(std::string("initialized: "));
    if (initialized_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    info.append(std::string("  learned: "));
    if (params_.is_learned_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    info.append(std::string("  setup: "));
    if (params_.is_setup_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    info.append(std::string("  is start set: "));
    if (params_.is_start_set_)
    {
        info.append(std::string("true "));
    }
    else
    {
        info.append(std::string("false"));
    }

    info.append(std::string("\n") + params_.getInfoString());
    for (int i = 0; i < params_.num_transformation_systems_; i++)
    {
        info.append(std::string("\n\t"));
        info.append(transformation_systems_[i].getInfoString());
    }

    return info;*/
}

}
