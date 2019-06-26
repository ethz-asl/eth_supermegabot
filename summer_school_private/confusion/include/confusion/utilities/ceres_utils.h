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

#ifndef CONFUSION_CERES_UTILS_H_
#define CONFUSION_CERES_UTILS_H_

#include <Eigen/Geometry>
#include <ceres/ceres.h>
#include <confusion/utilities/Pose.h>

namespace confusion {

//A collection of random helper functions to use with Ceres Solver

//Easy way to extract the raw double values of ceres::Jet types
void getDouble(const double& in, double& out) {
	out = in;
}
template <int N>
void getDouble(const ceres::Jet<double,N>& in, double& out) {
	out = in.a;
}
double getDouble(const double in) {
	return in;
}
template <int N>
double getDouble(const ceres::Jet<double,N> in) {
	return in.a;
}

void getDoubles(const double* in, int dim, double* out) {
	for (int i=0; i<dim; ++i) {
		out[i] = in[i];
	}
}
template <int N>
void getDoubles(const ceres::Jet<double,N>* in,
		int dim, double* out) {
	for (int i=0; i<dim; ++i) {
		out[i] = in[i].a;
	}
}

//Easy way to extract the raw double values of a pose in a cost function (for example to print the values with std::cout)
Pose<double> getDoublesPose(const Pose<double> P) {
	return P;
}

template <int N>
Pose<double> getDoublesPose(const Pose<ceres::Jet<double,N>> P) {
	Pose<double> Pout;
	getDoubles(P.trans.data(), 3, Pout.trans.data());
	getDoubles(P.rot.coeffs().data(), 4, Pout.rot.coeffs().data());
	return Pout;
}

//Return true if successful
bool getCovariance(ceres::Problem& problem, const double* param1, const size_t param1_size,
		const double* param2, size_t param2_size, std::string ss, Eigen::MatrixXd* cov_out = nullptr) {
	ceres::Covariance::Options cov_options;
	ceres::Covariance covariance(cov_options);
	std::vector<std::pair<const double*, const double*> > covariance_blocks;
	covariance_blocks.push_back(std::make_pair(param1, param2));

	if(covariance.Compute(covariance_blocks, &problem)) {
		double cov_array[param1_size * param2_size];
		covariance.GetCovarianceBlockInTangentSpace(param1, param2, cov_array); //todo Okvis requires an older version of ceres which doesn't have this function. This is a quick hack for additional testing. Fix this later! Either consider the local parameterization manually or move ahead the ceres version again once I dont need okvis
//		covariance.GetCovarianceBlock(param1, param2, cov_array);
		Eigen::Map<Eigen::MatrixXd> cov(cov_array, param1_size, param2_size);
//		std::cout << ss << " cov:\n" << cov << std::endl;

		if (cov_out)
			*cov_out = cov;

		return true;
	}
	else {
		std::cout << "Unable to compute covariance " << ss << std::endl;
	}

	return false;
}

//Build a dense eigen matrix from the ceres CRSMatrix datatype
template <typename T>
void buildDenseMatrix(const ceres::CRSMatrix Ain, Eigen::Matrix<T,Eigen::Dynamic,Eigen::Dynamic>& A) {
	A.resize(Ain.num_rows, Ain.num_cols);
	A.setZero();
	for (int r=0; r<Ain.num_rows; ++r) {
		for (int i=Ain.rows[r]; i<Ain.rows[r+1]; ++i) {
			A(r,Ain.cols[i]) = T(Ain.values[i]);
		}
	}
}


/**
 * Protect divide by zero with the sqrt of an auto-diff type.
 * This effectively truncates the Jacobian of sqrt(x) as x->0
 * The value of sqrt(x=0) is not impacted
 * @param error
 */
void safeSqrt(double& error) {
	if (error < 1e-10)
		error = 1e-10;
}
template <int N>
void safeSqrt(ceres::Jet<double,N>& error) {
	if (error.a < 1e-10)
		error.a = 1e-10;
}

} //namespace confusion

#endif /* CONFUSION_CERES_UTILS_H_ */
