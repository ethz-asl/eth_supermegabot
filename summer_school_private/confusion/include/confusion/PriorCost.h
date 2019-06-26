/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_CONFUSION_PRIOR_COST_H
#define INCLUDE_CONFUSION_PRIOR_COST_H


#include <memory>
#include <Eigen/Core>
#include "confusion/utilities/distances.h"
#include "confusion/utilities/ceres_utils.h"
#include "confusion/Parameter.h"

//Constrains the start of the batch, given the marginalized value of the leading
//state and the corresponding confidence in the estimate.
//Calculates the error in tangent space

namespace confusion {

class PriorCost : public ceres::CostFunction {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	PriorCost(const std::vector<FixedParameter>& marginalizedParams,
			const Eigen::MatrixXd& W_, Eigen::VectorXd& priorErrorOffset_)
			: marginalizedParams_(marginalizedParams), W(W_), priorErrorOffset(priorErrorOffset_) {
		std::vector<int>* param_sizes = mutable_parameter_block_sizes();
		int priorParameterLocalDim = 0;

		//Do the bookkeeping on the parameter sizes for ceres
		for (const auto &param: marginalizedParams) {
			param_sizes->push_back(param.size());
			priorParameterLocalDim += param.localSize();
		}

		//Make sure the sizes match
		if (priorParameterLocalDim != W.rows() || priorParameterLocalDim != W.cols()) {
			std::cout << "---Size mismatch in PriorCost! W size:" << W.rows() << "," << W.cols() << "; expected to be of size: " <<  priorParameterLocalDim << std::endl;
		}
		if (W.rows() != priorErrorOffset.rows()) {
			std::cout << "---Size of W and priorErrorOffset don't match! W size: " <<
					W.rows() << "; priorErrorOffset size: " << priorErrorOffset.rows() << std::endl;
		}

		errorSize = priorParameterLocalDim;

		//Set the number of residuals for ceres
		set_num_residuals(errorSize);

//std::cout << "W:\n" << W << "\n priorErrorOffset: " << priorErrorOffset.transpose() << std::endl;

#ifdef DEBUG_PRIOR
		std::cout << "PriorCost created with " << marginalizedParams_.size() <<
				" marginalized parameters of " << errorSize << " local size " << std::endl;
#endif
	}

	~PriorCost() {}

	bool Evaluate(double const* const* x, double* residuals, double** jacobians) const {
#ifdef COST_DEBUG
		std::cout << "Starting PriorCost" << std::endl;
#endif
		size_t residualIndex = 0;

		Eigen::Map<Eigen::VectorXd> e(residuals, errorSize);

		//Evaluate for the static parameters
		for (size_t i=0; i<marginalizedParams_.size(); ++i) {
//std::cout << "i=" << i << "; errorSize: " << errorSize << std::endl;
			if (!marginalizedParams_[i].isConstant()) {
				if (marginalizedParams_[i].parameterization_) {
//std::cout << "i=" << i << "; p0_check=" << marginalizedParams_[i].copyOfData_[0] << "; p0=" << x[0][0] << std::endl;
					marginalizedParams_[i].parameterization_->boxMinus(
							marginalizedParams_[i].copyOfData_.data(), x[i],
							e.data()+residualIndex);
					if (jacobians && jacobians[i]) {
//std::cout << "getting local jacobian for param " << i << " of size " << marginalizedParams_[i].size() << " and local size " << marginalizedParams_[i].localSize() << std::endl;
						Eigen::MatrixXd dd_dxi =
								marginalizedParams_[i].parameterization_->boxMinusJacobianRight(
								marginalizedParams_[i].copyOfData_.data(), x[i]);
//std::cout << "dd_dxi: " << dd_dxi << std::endl;
						Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > de_dxi(
								jacobians[i], errorSize, marginalizedParams_[i].size());
//std::cout << "W portion: " << W.block(0, residualIndex, errorSize, marginalizedParams_[i].localSize()) << std::endl;
						de_dxi = W.block(0, residualIndex, errorSize, marginalizedParams_[i].localSize()) * dd_dxi;
					}
				}
				else {
					//No local parameterization
					for (size_t j=0; j<marginalizedParams_[i].size(); ++j) {
						e[residualIndex+j] = marginalizedParams_[i].copyOfData_(j) - x[i][j];
//std::cout << "xi_param=" << x[i][j] << "; xi_marg=" << marginalizedParams_[i].copyOfData_(j) << std::endl;
					}
					if (jacobians && jacobians[i]) {
//std::cout << "getting jacobian for param " << i << std::endl;
						//The Jacobian is W.col[xi]*Identity
						Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> > de_dxi(
								jacobians[i], errorSize, marginalizedParams_[i].size());
						de_dxi = -1.0 * W.block(0, residualIndex, errorSize, marginalizedParams_[i].size());
					}
				}
				residualIndex += marginalizedParams_[i].localSize();
			}
		}

		//Do some sanity checks
		if (residualIndex != errorSize)
			std::cout << "ERROR! PriorCost residualIndex is not equal to errorSize after cost computation! actual size: " << residualIndex << ", expected size: " << errorSize << std::endl;
#ifdef PRINT_PRIOR_COST
		std::cout << "PriorCost before weighting: " << e.transpose() << std::endl;
#endif

		//Apply the weighting
		e = priorErrorOffset + W * e;

#ifdef PRINT_PRIOR_COST
		std::cout << "PriorCost: " << e.transpose() << std::endl;
#endif
#ifdef COST_DEBUG
		std::cout << "Done PriorCost" << std::endl;
//		std::cout << "PriorCost: " << e.transpose() << std::endl;
#endif
		return true;
	}

	const std::vector<FixedParameter>& marginalizedParams_; //These are copied in here so that they maintain their values during optimization
	const Eigen::MatrixXd& W;
	const Eigen::VectorXd& priorErrorOffset; //The prior error from the previous MHE problem
	size_t errorSize;

//	std::vector<double*> priorParams_; //This is sent at problem.AddResidualBlock to tell Ceres which parameters are involved
};

} // namespace confusion

#endif // INCLUDE_CONFUSION_PRIOR_COST_H
