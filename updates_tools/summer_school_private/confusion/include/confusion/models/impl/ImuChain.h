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


bool evaluateImuChainCost(ImuChain* imuChain, double const* const* x,
		double* residuals, double** jacobians) {
#ifdef COST_DEBUG
	std::cout << "Starting IMU cost computation at t=" << imuChain->tStart() << " with " << imuChain->measurements_.size() << " measurements" << std::endl;
#endif
#ifdef IMU_COST_DEBUG
	std::cout << "Starting IMU cost computation" << std::endl;
#endif

//	bool extraPrint = false;
//	if (imuChain->measurements_.size() < 5)
//		extraPrint = true;

	//x contains the previous (x[0] to x[4]) and the current state (x[5] to x[9]),
	Eigen::Matrix<double,3,1> p_i_prev(x[0]);
	Eigen::Quaterniond q_i_prev = Eigen::Map<const Eigen::Quaterniond>(x[1]);
	Eigen::Matrix<double,3,1> v_i_prev(x[2]);
	Eigen::Matrix<double,3,1> b_a_prop(x[3]);
	Eigen::Matrix<double,3,1> b_g_prop(x[4]);

	//Get the current estimate of the gravity vector
	auto firstMeasPtr = std::dynamic_pointer_cast<ImuMeas>(imuChain->measurements_.front());
	Eigen::Vector3d g_w;
	if (imuChain->optGravity_)
		g_w = confusion::gravityVec(x[10], firstMeasPtr->imuCalibration_->gravityMagnitude_);
	else
		g_w = firstMeasPtr->imuCalibration_->g_w_;

	Eigen::Matrix<double,3,1> p_i_prop = p_i_prev;
	Eigen::Quaterniond q_i_prop = q_i_prev;
	Eigen::Matrix<double,3,1> v_i_prop = v_i_prev;

//	if (extraPrint) {
//		std::cout << "IMU cost with few measurements. tStart=" << imuChain->tStart() << ", tEnd=" << imuChain->tEnd() << " meas at: ";
//		for (auto &meas: imuChain->measurements_)
//			std::cout << meas->t() << ", ";
//		std::cout << std::endl;
//		std::cout << "Before p: " << p_i_prop.transpose() << "; q: " << q_i_prop.coeffs().transpose() <<
//							"; v: " << v_i_prop.transpose() << "; ba: " << b_a_prop.transpose() << "; bg: " << b_g_prop.transpose() << std::endl;
//	}

	std::deque<std::shared_ptr<ProcessMeasurement>>::const_iterator imu_current;
	imu_current = imuChain->measurements_.begin();

	//Start with no state uncertainty
	Eigen::Matrix<double,16,16> cov_x(Eigen::Matrix<double,16,16>::Zero());// = cov_x0; //We have the option to specify the leading covariance
	Eigen::Matrix<double,16,16> dx1_dx0(Eigen::Matrix<double,16,16>::Identity());
	Eigen::Matrix<double,16,3> dx1_dgw(Eigen::Matrix<double,16,3>::Zero());

#ifdef IMU_COST_DEBUG
	std::cout << "Before p: " << p_i_prop.transpose() << "; q: " << q_i_prop.coeffs().transpose() <<
			"; v: " << v_i_prop.transpose() << "; ba: " << b_a_prop.transpose() << "; bg: " << b_g_prop.transpose() << std::endl;
#endif

	size_t numIntegrations = 0;
	if (imuChain->optGravity_) {
		//Propagate from ti_ to t_imu_1
		if ((imu_current+1) != imuChain->measurements_.end()) {
			if ((*imu_current)->t() <= imuChain->tStart()) {
				integrate_opt_gravity(p_i_prop, q_i_prop, v_i_prop, b_a_prop, b_g_prop,
					*imu_current, g_w, (*(imu_current+1))->t() - imuChain->tStart(), cov_x, dx1_dx0, dx1_dgw,
					firstMeasPtr->imuCalibration_->cov_imu_nominal_);
				++imu_current;
				++numIntegrations;
			}
			else {
				//This when there is no imu measurement preceeding ti_
				integrate_opt_gravity(p_i_prop, q_i_prop, v_i_prop, b_a_prop, b_g_prop,
					*imu_current, g_w, ((*imu_current)->t() - imuChain->tStart()), cov_x, dx1_dx0, dx1_dgw,
					firstMeasPtr->imuCalibration_->cov_imu_nominal_);
				++numIntegrations;
			}
		}

		//Iterate through the imu measurements
		while ((imu_current+1) != imuChain->measurements_.end() && (*(imu_current+1))->t() <= imuChain->tEnd()) {
			integrate_opt_gravity(p_i_prop, q_i_prop, v_i_prop, b_a_prop, b_g_prop,
				*imu_current, g_w, (*(imu_current+1))->t() - (*imu_current)->t(),
				cov_x, dx1_dx0, dx1_dgw, firstMeasPtr->imuCalibration_->cov_imu_nominal_);
			++imu_current;
			++numIntegrations;
		}

		//Propogate the last chunk of time. Don't do anything else if the last IMU measurement is directly at tj_
		if ((*imu_current)->t() < imuChain->tEnd()) {
			integrate_opt_gravity(p_i_prop, q_i_prop, v_i_prop, b_a_prop, b_g_prop,
				*imu_current, g_w, imuChain->tEnd() - (*imu_current)->t(), cov_x, dx1_dx0, dx1_dgw,
				firstMeasPtr->imuCalibration_->cov_imu_nominal_);
			++numIntegrations;
		}
	}
	else {
		//Propagate from ti_ to t_imu_1
		if ((imu_current+1) != imuChain->measurements_.end()) {
			if ((*imu_current)->t() <= imuChain->tStart()) {
				integrate(p_i_prop, q_i_prop, v_i_prop, b_a_prop, b_g_prop,
					*imu_current, g_w, (*(imu_current+1))->t() - imuChain->tStart(), cov_x, dx1_dx0,
					firstMeasPtr->imuCalibration_->cov_imu_nominal_);
				++imu_current;
				++numIntegrations;
			}
			else {
				//This when there is no imu measurement preceeding ti_
				integrate(p_i_prop, q_i_prop, v_i_prop, b_a_prop, b_g_prop,
					*imu_current, g_w, ((*imu_current)->t() - imuChain->tStart()), cov_x, dx1_dx0,
					firstMeasPtr->imuCalibration_->cov_imu_nominal_);
				++numIntegrations;
			}
		}

		//Iterate through the imu measurements
		while ((imu_current+1) != imuChain->measurements_.end() && (*(imu_current+1))->t() <= imuChain->tEnd()) {
			integrate(p_i_prop, q_i_prop, v_i_prop, b_a_prop, b_g_prop,
				*imu_current, g_w, (*(imu_current+1))->t() - (*imu_current)->t(),
				cov_x, dx1_dx0, firstMeasPtr->imuCalibration_->cov_imu_nominal_);
			++imu_current;
			++numIntegrations;
		}

		//Propogate the last chunk of time. Don't do anything else if the last IMU measurement is directly at tj_
		if ((*imu_current)->t() < imuChain->tEnd()) {
			integrate(p_i_prop, q_i_prop, v_i_prop, b_a_prop, b_g_prop,
				*imu_current, g_w, imuChain->tEnd() - (*imu_current)->t(), cov_x, dx1_dx0,
				firstMeasPtr->imuCalibration_->cov_imu_nominal_);
			++numIntegrations;
		}
	}

	if (numIntegrations < 2) {
		//We need to give a small non-zero uncertainty to the IMU position if only one
		//measurement is linked even though there should actually be no constraint
		//in the position according to the measurement model.
		cov_x.topLeftCorner<3,3>().setIdentity();
		cov_x.topLeftCorner<3,3>() *= 1e-22;
	}

#if defined IMU_COST_DEBUG
	std::cout << "After p: " << p_i_prop.transpose() << "; q: " << q_i_prop.coeffs().transpose() <<
			"; v: " << v_i_prop.transpose() << "; ba: " << b_a_prop.transpose() << "; bg: " << b_g_prop.transpose() << std::endl;
#endif

	//Get error from the propagated state to the current estimate
	Eigen::Map<Eigen::Matrix<double,15,1> > e(residuals);
	confusion::VectorDistance(p_i_prop.data(), x[5], e.data());
	confusion::QuatDistance(q_i_prop, Eigen::Quaterniond(x[6]), e.data()+3);
	confusion::VectorDistance(v_i_prop.data(), x[7], e.data()+6);
	confusion::VectorDistance(b_a_prop.data(), x[8], e.data()+9);
	confusion::VectorDistance(b_g_prop.data(), x[9], e.data()+12);
//	std::cout << "e_raw: " << e.transpose() << std::endl;

	//Partial derivative of the error wrt the propogated state
	Eigen::Matrix<double,15,16> de_dx1p(Eigen::Matrix<double,15,16>::Zero());
	de_dx1p.block<3,3>(0,0).setIdentity();
	de_dx1p.block<9,9>(6,7).setIdentity();
	Eigen::Matrix<double,3,4> de_dqm_left;
	confusion::calc_de_dq_left(q_i_prop, Eigen::Quaterniond(x[6]), de_dqm_left);
	de_dx1p.block<3,4>(3,3) = de_dqm_left;
//	std::cout << "de_dx1p:\n" << de_dx1p << std::endl;
//	std::cout << "dx1_dx0:\n" << dx1_dx0 << std::endl;

	//Partial derivative of the error wrt the estimated state
	Eigen::Matrix<double,15,16> de_dx1 = -1.0 * de_dx1p;
	Eigen::Matrix<double,3,4> de_dqm_right;
	confusion::calc_de_dq_right(q_i_prop, Eigen::Quaterniond(x[6]), de_dqm_right);
	de_dx1.block<3,4>(3,3) = de_dqm_right;

#ifndef IMU_UNIT_TEST
	Eigen::Matrix<double,15,15> cov_e = de_dx1p * cov_x * de_dx1p.transpose();
#endif

//	std::cout << "de_dx1p out:\n" << de_dx1p << std::endl << std::endl;
//	std::cout << "cov_x out:\n" << cov_x << std::endl << std::endl;
//	std::cout << "cov_e out:\n" << cov_e << std::endl << std::endl;

	//For unit testing, we set the weighting to the identity. This is because the assuption that the weighting is constant breaks the unit test.
#ifdef IMU_UNIT_TEST
//	std::cout << "\n\nIMU_UNIT_TEST turned ON. If you are not running a unit test, this is a problem.\n\n" << std::endl;
	Eigen::Matrix<double,15,15> S(Eigen::Matrix<double,15,15>::Identity());
#else
	//We need the stiffness matrix: cov^(-1/2)
	Eigen::Matrix<double,15,15> inf = cov_e.inverse();
	inf = 0.5 * inf + 0.5 * inf.transpose().eval();
	Eigen::LLT<Eigen::Matrix<double,15,15>> llt(inf);
	Eigen::Matrix<double,15,15> S = llt.matrixL().transpose();
//	std::cout << "S:\n" << S << std::endl;
#endif

	//Weigh errors by the stiffness
	e = S * e;

//	if (extraPrint) {
//		std::cout << "After p: " << p_i_prop.transpose() << "; q: " << q_i_prop.coeffs().transpose() <<
//							"; v: " << v_i_prop.transpose() << "; ba: " << b_a_prop.transpose() << "; bg: " << b_g_prop.transpose() << std::endl;
//		std::cout << "cov_x for " << imuChain->measurements_.size() << " meas:\n" << cov_x << std::endl;
////		std::cout << "cov_e:\n" << cov_e << std::endl;
//		std::cout << "S:\n" << S << std::endl;
////		std::cout << "e: " << e.transpose() << std::endl;
//	}

	//Fill in Jacobians
	if (jacobians) {
		Eigen::Matrix<double,15,16> c1 = S * de_dx1p;
		Eigen::Matrix<double,15,16> c2 = S * de_dx1;
		if (jacobians[0]) {
			//de_dp0
			Eigen::Map<Eigen::Matrix<double,15,3,Eigen::RowMajor> > de_dp0(jacobians[0]);
			de_dp0 = c1 * dx1_dx0.block<16,3>(0,0);
		}
		if (jacobians[1]) {
			//de_dq0
			//Need to reorder columns [x,y,z,w]!
			Eigen::Matrix<double,15,4> de_dq0_temp = c1 * dx1_dx0.block<16,4>(0,3);
			Eigen::Map<Eigen::Matrix<double,15,4,Eigen::RowMajor> > de_dq0(jacobians[1]);
			de_dq0.block<15,3>(0,0) = de_dq0_temp.block<15,3>(0,1);
			de_dq0.block<15,1>(0,3) = de_dq0_temp.block<15,1>(0,0);
		}
		if (jacobians[2]) {
			//de_dv0
			Eigen::Map<Eigen::Matrix<double,15,3,Eigen::RowMajor> > de_dv0(jacobians[2]);
			de_dv0 = c1 * dx1_dx0.block<16,3>(0,7);
		}
		if (jacobians[3]) {
			//de_dba0
			Eigen::Map<Eigen::Matrix<double,15,3,Eigen::RowMajor> > de_dba0(jacobians[3]);
			de_dba0 = c1 * dx1_dx0.block<16,3>(0,10);
		}
		if (jacobians[4]) {
			//de_dbg0
			Eigen::Map<Eigen::Matrix<double,15,3,Eigen::RowMajor> > de_dbg0(jacobians[4]);
			de_dbg0 = c1 * dx1_dx0.block<16,3>(0,13);
		}
		if (jacobians[5]) {
			//de_dp1
			Eigen::Map<Eigen::Matrix<double,15,3,Eigen::RowMajor> > de_dp1(jacobians[5]);
			de_dp1 = c2.block<15,3>(0,0);
//std::cout << "dei_dp1: " << de_dp1 << std::endl;
		}
		if (jacobians[6]) {
			//de_dq1
			//Need to reorder columns [x,y,z,w] because Ceres sees the quat data in that order!
			Eigen::Matrix<double,15,4> de_dq1_temp = c2.block<15,4>(0,3);
			Eigen::Map<Eigen::Matrix<double,15,4,Eigen::RowMajor> > de_dq1(jacobians[6]);
			de_dq1.block<15,3>(0,0) = de_dq1_temp.block<15,3>(0,1);
			de_dq1.block<15,1>(0,3) = de_dq1_temp.block<15,1>(0,0);
//			de_dq1 = de_dq1_temp;
		}
		if (jacobians[7]) {
			//de_dv1
			Eigen::Map<Eigen::Matrix<double,15,3,Eigen::RowMajor> > de_dv1(jacobians[7]);
			de_dv1 = c2.block<15,3>(0,7);
		}
		if (jacobians[8]) {
			//de_dba
			Eigen::Map<Eigen::Matrix<double,15,3,Eigen::RowMajor> > de_dba(jacobians[8]);
			de_dba = c2.block<15,3>(0,10);
		}
		if (jacobians[9]) {
			//de_dbg
			Eigen::Map<Eigen::Matrix<double,15,3,Eigen::RowMajor> > de_dbg(jacobians[9]);
			de_dbg = c2.block<15,3>(0,13);
		}
		if (imuChain->optGravity_ && jacobians[10]) {
			//de_dgw
			Eigen::Matrix<double,15,3> de_dgw = c1 * dx1_dgw;

			//Need to relate this to the gravity vector rotation angles
			Eigen::Matrix<double,3,2> dg_dr = confusion::gravityVec_jacob(x[10], firstMeasPtr->imuCalibration_->gravityMagnitude_);
			Eigen::Map<Eigen::Matrix<double,15,2,Eigen::RowMajor> > de_dr(jacobians[10]);
			de_dr = de_dgw * dg_dr;
		}
	}

#ifdef COST_DEBUG
//	std::cout << "Done ImuCost" << std::endl;
	std::cout << "IMU cost for " << imuChain->measurements_.size() << " meas = [" << e.transpose() << "]" << std::endl;
#endif

	return true;
}
