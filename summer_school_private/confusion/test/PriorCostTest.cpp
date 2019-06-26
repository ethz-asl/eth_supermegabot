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


//#define OPT_DEBUG
//#define PRINT_PRIOR_COST

#include <gtest/gtest.h>
#include "confusion/Parameter.h"
#include "confusion/PriorConstraint.h"
#include "confusion/StaticParameterVector.h"
//#include "include_test/ImuStateR.h"
#include "confusion/utilities/rotation_utils.h"

using namespace confusion;

const double tolerance = 5e-8;
static const int errorSize = 23;

void check_quats(Eigen::Quaterniond q1, Eigen::Quaterniond q2, Eigen::Vector4d dq_est, bool print = false) {
	//Print results
	Eigen::Quaterniond q2_est(q1.w()+dq_est(0),q1.x()+dq_est(1),q1.y()+dq_est(2),q1.z()+dq_est(3));

	if (print) {
		std::cout << "q1 =[" << q1.w() << "," << q1.x() << "," << q1.y() << "," << q1.z() << "]" << std::endl;
		std::cout << "q2 =[" << q2.w() << "," << q2.x() << "," << q2.y() << "," << q2.z() << "]" << std::endl;
		std::cout << "q2e=[" << q2_est.w() << "," << q2_est.x() << "," << q2_est.y() << "," << q2_est.z() << "]" << std::endl;
	}

	Eigen::Vector4d dq;
	dq << q2.w()-q1.w(), q2.x()-q1.x(), q2.y()-q1.y(), q2.z()-q1.z();
//	std::cout << "test: " << dq(0)-dq_est(0) << "," << dq(1)-dq_est(1) << "," <<
//			dq(2)-dq_est(2) << "," << dq(3)-dq_est(3) << std::endl;
	EXPECT_LT(fabs(dq(0)-dq_est(0)),tolerance);
	EXPECT_LT(fabs(dq(1)-dq_est(1)),tolerance);
	EXPECT_LT(fabs(dq(2)-dq_est(2)),tolerance);
	EXPECT_LT(fabs(dq(3)-dq_est(3)),tolerance);
}


TEST(priorCostTest, ShouldPass) {
	for (int i=0; i<10; ++i) {
		std::vector<confusion::Parameter> stateParameters;
		Pose<double> T_w_i;
		T_w_i.trans.setRandom();
		T_w_i.rot.coeffs().setRandom();
		T_w_i.rot.coeffs().normalize();
		Eigen::Vector3d linVel;
		linVel.setRandom();
		Eigen::Vector3d accelBias;
		accelBias.setRandom();
		Eigen::Vector3d gyroBias;
		gyroBias.setRandom();
		stateParameters.emplace_back(confusion::Parameter(T_w_i.trans.data(), 3, "t_w_i"));
		stateParameters.back().setInitialConstraintWeighting(Eigen::Matrix3d::Random());
		stateParameters.emplace_back(confusion::Parameter(T_w_i.rot.coeffs().data(), 4, "q_w_i", std::make_shared<QuatParam>()));
		stateParameters.back().setInitialConstraintWeighting(Eigen::Matrix3d::Random());
		stateParameters.emplace_back(confusion::Parameter(linVel.data(), 3, "linVel"));
		stateParameters.back().setInitialConstraintWeighting(Eigen::Matrix3d::Random());
		stateParameters.emplace_back(confusion::Parameter(accelBias.data(), 3, "b_a"));
		stateParameters.back().setInitialConstraintWeighting(Eigen::Matrix3d::Random());
		stateParameters.emplace_back(confusion::Parameter(gyroBias.data(), 3, "b_g"));
		stateParameters.back().setInitialConstraintWeighting(Eigen::Matrix3d::Random());

		//Create some dummy static parameters
		Eigen::Vector2d param2cdata;
		param2cdata.setRandom();
		Eigen::Vector2d param2data;
		param2data.setRandom();
		Eigen::Vector2d param2_other_data;
		param2_other_data.setRandom();
		Pose<double> paramPose;
		paramPose.trans.setRandom();
		paramPose.rot.coeffs().setRandom();
		paramPose.rot.coeffs().normalize();
		StaticParameterVector staticParameters;
		confusion::Parameter param2(param2data.data(), 2, "param2");
		param2.setInitialConstraintWeighting(Eigen::Matrix2d::Random());
		confusion::Parameter param2_const(param2cdata.data(), 2, "param2_const", true);
		confusion::Parameter param2_other(param2_other_data.data(), 2, "param2_other");
		staticParameters.addParameter(param2);
		staticParameters.addParameter(param2_const);
		staticParameters.addParameter(param2_other); //This one isnt immediately added to the prior problem
		confusion::Parameter transParam(paramPose.trans.data(), 3, "transParam");
		transParam.setInitialConstraintWeighting(Eigen::Matrix3d::Random());
		staticParameters.addParameter(transParam);
		confusion::Parameter rotParam(paramPose.rot.coeffs().data(), 4, "rot", std::make_shared<QuatParam>());
		rotParam.setInitialConstraintWeighting(Eigen::Matrix3d::Random());
		staticParameters.addParameter(rotParam);

		//Create the PriorConstraint and add all of the parameters
		PriorConstraint priorConstraint;
		for (int i=0; i<stateParameters.size(); ++i) {
	//		if (stateParameters[i].isActive() && stateParameters[i].immediatelyAddToPrior()) {
#ifdef OPT_DEBUG
				std::cout << "Adding the first state to the prior constraint. i=" << i << std::endl;
#endif
				priorConstraint.addStateParameterAndActivate(stateParameters[i]);
	//		}
		}
		priorConstraint.fixMarginalizedParams();
		priorConstraint.initialize();

		//Store the marginalized parameters for comparison later
		Pose<double> T_w_i_marg = T_w_i;
		Eigen::Vector3d linVel_marg = linVel;
		Eigen::Vector3d accelBias_marg = accelBias;
		Eigen::Vector3d gyroBias_marg = gyroBias;
		Eigen::Vector2d param2data_marg = param2data;
		Pose<double> paramPose_marg = paramPose;

		for (auto& iter: staticParameters) {
			if (//iter.second.isActive() &&
					iter.second.immediatelyAddToPrior() &&
					!iter.second.isConstant()) {
				//Add the parameter to the prior constraint the first time it goes active
				//todo Make this a separate function in the prior constraint?
				priorConstraint.addStaticParameter(iter.second);
#ifdef OPT_DEBUG
				std::cout << "Added static parameter with size " << iter.second.size() <<
						" to the problem and immediately activated it." << std::endl;
#endif
			}
		}

		//Make sure that the prior constraint parameter sizes make sense
		EXPECT_EQ(priorConstraint.priorCostWeighting_.rows(),errorSize);
		EXPECT_EQ(priorConstraint.priorCostWeighting_.cols(),errorSize);
		EXPECT_EQ(priorConstraint.priorErrorOffset_.rows(),errorSize);

		//Set a random confidence in the prior parameters
		priorConstraint.priorCostWeighting_.setIdentity(); //Random();
		priorConstraint.priorErrorOffset_.setZero(); //To test if the parameters get back to their original values

//		//Make sure the priorCostWeighting is full rank
//		Eigen::MatrixXd weighting_offset(errorSize,errorSize);
//		weighting_offset.setIdentity();
////		weighting_offset *= 1e-3;
//		priorConstraint.priorCostWeighting_ = priorConstraint.priorCostWeighting_ * weighting_offset;

		//Perturb the state to simulate some optimizer action
		T_w_i.trans.setRandom();
		T_w_i.rot.coeffs().setRandom();
		T_w_i.rot.coeffs().normalize();
		linVel.setRandom();
		accelBias.setRandom();
		gyroBias.setRandom();
		param2data.setRandom();
		paramPose.trans.setRandom();
		paramPose.rot.coeffs().setRandom();
		paramPose.rot.coeffs().normalize();

        ceres::Problem::Options problemOptions_;
        problemOptions_.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
        problemOptions_.local_parameterization_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

        ceres::Problem problem(problemOptions_);
        ceres::Solver::Options options;
        options.minimizer_progress_to_stdout = false;
        options.update_state_every_iteration = true;
        options.parameter_tolerance = 1e-10;
        options.gradient_tolerance = 1e-15;
//		options.max_num_iterations = 10; //todo Only for debugging!

		//Create the prior cost
		priorConstraint.addPriorCostToProblem(&problem);
		Eigen::VectorXd e(errorSize);

		std::vector<double*> jacobians;
		Eigen::Matrix<double,errorSize,3,Eigen::RowMajor> de_dt;
		Eigen::Matrix<double,errorSize,4,Eigen::RowMajor> de_dq;
		Eigen::Matrix<double,errorSize,3,Eigen::RowMajor> de_dv;
		Eigen::Matrix<double,errorSize,3,Eigen::RowMajor> de_dba;
		Eigen::Matrix<double,errorSize,3,Eigen::RowMajor> de_dbg;
		Eigen::Matrix<double,errorSize,2,Eigen::RowMajor> de_dp2;
		Eigen::Matrix<double,errorSize,3,Eigen::RowMajor> de_dpt;
		Eigen::Matrix<double,errorSize,4,Eigen::RowMajor> de_dpq;
		jacobians.push_back(de_dt.data());
		jacobians.push_back(de_dq.data());
		jacobians.push_back(de_dv.data());
		jacobians.push_back(de_dba.data());
		jacobians.push_back(de_dbg.data());
		jacobians.push_back(de_dp2.data());
		jacobians.push_back(de_dpt.data());
		jacobians.push_back(de_dpq.data());

        priorConstraint.priorCostPtr->Evaluate(priorConstraint.priorParams_.data(),e.data(),jacobians.data());

		//Perturb the estimated parameters
		Eigen::Vector3d dt(Eigen::Vector3d::Random());
		dt *= 1e-4;
		T_w_i.trans += dt;

		Eigen::Vector4d dq(Eigen::Vector4d::Random());
		dq *= 1e-4;
		T_w_i.rot.coeffs() += dq;

		Eigen::Vector2d dp2(Eigen::Vector2d::Random());
		dp2 *= 1e-4;
		param2data += dp2;

		Eigen::Vector3d dpt(Eigen::Vector3d::Random());
		dpt *= 1e-4;
		paramPose.trans += dpt;

		Eigen::Vector4d dpq(Eigen::Vector4d::Random());
		dpq *= 1e-4;
		paramPose.rot.coeffs() += dpq;

		Eigen::VectorXd e_pert(errorSize);
		priorConstraint.priorCostPtr->Evaluate(priorConstraint.priorParams_.data(),e_pert.data(),nullptr);

//		std::cout << "e: " << e.transpose() << "\ne_pert: " << e_pert.transpose() << std::endl;

		Eigen::VectorXd de = e_pert - e;
		Eigen::VectorXd de_est = de_dt * dt + de_dq * dq + de_dp2 * dp2 + de_dpt * dpt + de_dpq * dpq;

//		std::cout << "de = " << de.transpose() << "\nde_est = " << de_est.transpose() << "\n" << std::endl;

	//	Eigen::Vector3d dqm = -dq;
	//	T_w_i_.rot.coeffs() -= dq; //T_w_i_.rot = quaternionBoxPlus(T_w_i_.rot, dqm);

		for (int i=0; i<e.size(); ++i)
			EXPECT_LT(fabs(de(i)-de_est(i)),tolerance);



		//Try optimizing over the cost to get back to the marginalized values
		//Add the state parameters to the problem
		for(int i=0; i<stateParameters.size(); ++i)
			stateParameters[i].addToProblem(&problem);

		//Add the static parameters to the problem
		staticParameters.getParameter(param2data.data())->addToProblem(&problem);
		staticParameters.getParameter(paramPose.trans.data())->addToProblem(&problem);
		staticParameters.getParameter(paramPose.rot.coeffs().data())->addToProblem(&problem);

		//Solve
		ceres::Solver::Summary summary;
		Solve(options, &problem, &summary);

//		std::cout << summary.FullReport() << std::endl;

		//The parameters should get back to the marginalized values
		//todo Commented out because this test fails due to flips in quaternions!!!
		for (int i=0; i<priorConstraint.marginalizedParams_.size(); ++i) {
//			std::cout << "marg: " << priorConstraint.marginalizedParams_[i].data().transpose() << "; est: ";
			for (int j=0; j<priorConstraint.marginalizedParams_[i].size(); ++j) {
//				std::cout << priorConstraint.priorParams_[i][j] << ",";
//				EXPECT_LT(fabs(priorConstraint.marginalizedParams_[i].data()(j)-priorConstraint.priorParams_[i][j]),tolerance);
			}
//			std::cout << std::endl;
		}

		Eigen::Matrix<double,6,1> dPose = T_w_i_marg.distanceGlobal(T_w_i);
		Eigen::Matrix<double,6,1> dPose2 = T_w_i_marg.distance(T_w_i);
//		std::cout << "dPose: " << dPose.transpose() << std::endl;
//		std::cout << "dPose2: " << dPose2.transpose() << std::endl;
		for (int i=0; i<6; ++i)
			EXPECT_LT(fabs(dPose(i)),tolerance);

		Eigen::Matrix<double,6,1> dParamPose = paramPose_marg.distanceGlobal(paramPose);
		Eigen::Matrix<double,6,1> dParamPose2 = paramPose_marg.distance(paramPose);
//		std::cout << "dParamPose: " << dParamPose.transpose() << std::endl;
//		std::cout << "dParamPose2: " << dParamPose2.transpose() << std::endl;
		for (int i=0; i<6; ++i)
			EXPECT_LT(fabs(dParamPose(i)),tolerance);

//		std::cout << "dParam2: " << (param2data-param2data_marg).transpose() << std::endl;
		EXPECT_LT(fabs(param2data(0)-param2data_marg(0)),tolerance);
		EXPECT_LT(fabs(param2data(1)-param2data_marg(1)),tolerance);

//		std::cout << "dlinVel: " << (linVel-linVel_marg).transpose() << std::endl;
		for (int i=0; i<3; ++i)
			EXPECT_LT(fabs(linVel(i)-linVel_marg(i)),tolerance);

//		std::cout << "daccelBias: " << (accelBias-accelBias_marg).transpose() << std::endl;
		for (int i=0; i<3; ++i)
			EXPECT_LT(fabs(accelBias(i)-accelBias_marg(i)),tolerance);

//		std::cout << "dgyroBias: " << (gyroBias-gyroBias_marg).transpose() << std::endl;
		for (int i=0; i<3; ++i)
			EXPECT_LT(fabs(gyroBias(i)-gyroBias_marg(i)),tolerance);

//		std::cout << std::endl;
	}
}

int main(int argc, char** argv) {
	srand(time(NULL));

	std::cout.precision(10);

    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}
