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

#ifndef INCLUDE_CONFUSION_UTILITIES_H_
#define INCLUDE_CONFUSION_UTILITIES_H_

#include <Eigen/Core>
#include <ceres/ceres.h>

namespace confusion {

/**
 * Iteration callback to tell ceres to stop solving the current problem because
 * either the node was terminated with ctrl-c or the boolean flag passed requests it
 */
class SolverOverrunCallback: public ceres::IterationCallback {
public:
	SolverOverrunCallback(const bool& abortSolver, const int minIterations = 0)
				: abortSolver_(abortSolver), minIterations_(minIterations) { }

    virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
    	//Allow the user to terminate the optimization if desired
    	if (abortSolver_ && summary.iteration >= minIterations_) {
				std::cout << "\n--------Solver stopping early because the next image is ready---------\n" << std::endl;
				return ceres::SOLVER_TERMINATE_SUCCESSFULLY;
    	}

    	return ceres::SOLVER_CONTINUE;
    }

    const bool& abortSolver_;
    const int minIterations_;
};

//H and b should be passed in the order H = [H_marg_marg, H marg_stay; H_stay_marg, H_stay_stay]
/**
 * Marginalize out part of the H dx = b problem using Gaussian elimination. margDim
 * rows are removed from the end of the dx vector, so H and b should already be
 * re-ordered as desired.
 *
 * \param H H matrix (dimension N x N)
 * \param b b vector (dimension N x 1)
 * \param margDim The number of rows of the state vector to marginalize
 * \param Wout Weighting matrix to be used in the prior constraint after marginalization
 * \param eout Error offset vector to be used in the prior constraint after marginalization
 * \param HstarOut Optionally output the resulting Hstar matrix, which can be used later to later factor the prior constraint
 * \param bstarOut Optionally output the resulting bstar vector, which can be used later to later factor the prior constraint
 */
bool schurMarginalize(const Eigen::MatrixXd& H, const Eigen::VectorXd& b, const size_t& margDim,
		Eigen::MatrixXd& Wout, Eigen::VectorXd& eout,
		Eigen::MatrixXd* HstarOut = nullptr, Eigen::VectorXd* bstarOut = nullptr) {
	bool res = true;
	size_t stayDim = H.rows() - margDim;

	//Invert the top-left corner of H. Could use a pre-conditioner here if required, but haven't needed it yet.
	Eigen::MatrixXd Hmm = 0.5 * (H.topLeftCorner(margDim,margDim) + H.topLeftCorner(margDim,margDim).transpose()).eval(); // Enforce symmetry
	Eigen::LLT<Eigen::MatrixXd> llt(Hmm);
	Eigen::MatrixXd ii(margDim,margDim); ii.setIdentity();
	Eigen::MatrixXd Hmm_inv = llt.solve(ii);
	if (llt.info() == Eigen::NumericalIssue) {
		std::cout << "\n-----Numerical issue inverting Hmm!-----\n" << std::endl;
		res = false;
		//todo Use SVD and force no update on unobserved parameter dimensions
	}

	//Compute Hstar
	Eigen::MatrixXd Hstar = H.bottomRightCorner(stayDim,stayDim) -
			H.bottomLeftCorner(stayDim,margDim) * Hmm_inv * H.topRightCorner(margDim,stayDim);
	if (HstarOut) {
		*HstarOut = Hstar;
	}

	//Compute bstar
	Eigen::VectorXd bstar = b.tail(stayDim) - H.bottomLeftCorner(stayDim,margDim) * Hmm_inv * b.head(margDim);
	if (bstarOut) {
		*bstarOut = bstar;
	}

	//Compute SVD of Hstar, using a preconditioner (copied from okvis)
//		Hstar = 0.5*(Hstar+Hstar.transpose()).eval(); //Make sure Hstar is symmetric
	Eigen::MatrixXd p = (Hstar.diagonal().array() > 1.0e-9).select(Hstar.diagonal().cwiseSqrt(),1.0e-3);
	Eigen::MatrixXd p_inv = p.cwiseInverse();

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(0.5  * p_inv.asDiagonal() * (Hstar + Hstar.transpose())  * p_inv.asDiagonal());
	if (saes.info() != Eigen::Success) {
		std::cout << "\n-----Eigendecomposition of Hstar was not successful while computing Schur complement! Aborting-----\n" << std::endl;
		std::cout << "Hstar:\n" << Hstar << std::endl;
		abort();
	}

	//Pre-conditioned SVD matrix inversion
	static const double epsilon = std::numeric_limits<double>::epsilon();
	double tolerance = epsilon * stayDim * saes.eigenvalues().array().maxCoeff();
	Eigen::MatrixXd S = (saes.eigenvalues().array() > tolerance).select(saes.eigenvalues().array(), 0);
	Eigen::MatrixXd Sinv = (saes.eigenvalues().array() > tolerance).select(saes.eigenvalues().array().inverse(), 0);
	Eigen::MatrixXd S_sqrt = S.cwiseSqrt();
	Eigen::MatrixXd Sinv_sqrt = Sinv.cwiseSqrt();
//std::cout << "tolerance: " << tolerance << "; eigenvalues: " << saes.eigenvalues().transpose() << std::endl;
//std::cout << "eigenvectors:\n" << saes.eigenvectors() << std::endl;
//std::cout << "p:\n" << p << std::endl;
//std::cout << "S_sqrt:\n" << S_sqrt << std::endl;
	//Effective jacobian J: from H = J^T J
	Wout = (p.asDiagonal() * saes.eigenvectors() * (S_sqrt.asDiagonal())).transpose();
//std::cout << "p: " << p.transpose() << std::endl;
//std::cout << "Error: " << Hstar - Wout.transpose() * Wout << std::endl;
//std::cout << "Wout:\n" << Wout << std::endl;
	//Constant error at marginalization point: -(J^T)^(-1)*bstar
	Eigen::MatrixXd Jinv_T = Sinv_sqrt.asDiagonal() * saes.eigenvectors().transpose() * p_inv.asDiagonal();
	eout = -1.0 * Jinv_T * bstar;
//std::cout << "eout:\n" << eout.transpose() << std::endl;

//	//Try using Choleshky decomposition instead
//	Hstar = 0.5 * Hstar + 0.5 * Hstar.transpose().eval(); //Ensure symmetric
//	Eigen::LLT<Eigen::MatrixXd,Eigen::Upper> llt2(Hstar);
//	Eigen::MatrixXd J_p = llt2.matrixU();
////std::cout << "Error:\n" << llt2.reconstructedMatrix() - Hstar << std::endl;
//	Wout = J_p;
//	Eigen::MatrixXd Jinv_T = (J_p * J_p.transpose()).inverse() * J_p;
//	eout = -1.0 * Jinv_T * bstar;
////std::cout << "row 33: " << Wout.row(33) << std::endl;
////std::cout << "Wout: size: " << Wout.rows() << "," << Wout.cols() << "\n" << Wout << std::endl;
////std::cout << "eout:\n" << eout.transpose() << std::endl;

	return res;
}

Eigen::PermutationMatrix<Eigen::Dynamic> getPermutation(const int dim, const std::vector<std::pair<int,int>>& segmentPositionAndSize) {
	Eigen::PermutationMatrix<Eigen::Dynamic> perm(dim);
	perm.indices().setConstant(-1);

	//Put the target section at the front
	int index = 0;
	for(auto& s: segmentPositionAndSize) {
		for(int i=0; i<s.second; ++i) {
			perm.indices()[s.first+i] = index;
			++index;
		}
	}

	//Everything else goes to the back in the same order
	for (int i=0; i<dim; ++i) {
		if (perm.indices()[i] < 0) {
			perm.indices()[i] = index;
			++index;
		}
	}

	return perm;
}

void reorderNormalEquations(Eigen::MatrixXd& Hstar, Eigen::VectorXd& bstar, const std::vector<std::pair<int,int>>& segmentPositionAndSize) {
	Eigen::PermutationMatrix<Eigen::Dynamic> perm = getPermutation(Hstar.rows(), segmentPositionAndSize);
//std::cout << "perm: " << perm.indices().transpose() << std::endl;
	Hstar = perm * Hstar * perm.transpose();
	bstar = perm * bstar;
}

} //namespace confusion

#endif /* INCLUDE_CONFUSION_UTILITIES_H_ */
