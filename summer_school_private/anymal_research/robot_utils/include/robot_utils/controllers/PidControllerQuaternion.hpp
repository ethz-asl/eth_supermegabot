#pragma once

#include <Eigen/Core>
#include <kindr/Core>

#include "parameter_handler/parameter_handler.hpp"

namespace robot_utils {

    class PidControllerQuaternion {
    public:
        PidControllerQuaternion();

        PidControllerQuaternion(const Eigen::Vector3d maxEffort,
                                const Eigen::Vector3d kp,
                                const Eigen::Vector3d ki,
                                const Eigen::Vector3d kd);

        virtual ~PidControllerQuaternion();

        /**
         * @brief   Update the quaternion PID
         * q_BI     Rotates quantity in frame I to frame B
         * B_w_IB   Angular velocity of B w.r.t. frame I expressed in frame B
         */
        Eigen::Vector3d update(const double dt, const kindr::RotationQuaternionPD q_BI_des,
                               const kindr::RotationQuaternionPD q_BI_meas,
                               const kindr::LocalAngularVelocityPD B_w_IB_des,
                               const kindr::LocalAngularVelocityPD B_w_IB_meas);

        Eigen::Vector3d update(const double dt, const kindr::RotationQuaternionPD q_BI_des,
                               const kindr::RotationQuaternionPD q_BI_meas,
                               const kindr::LocalAngularVelocityPD B_w_IB_des);

        Eigen::Vector3d update(const double dt,
                               const kindr::RotationQuaternionPD q_BI_des,
                               const kindr::RotationQuaternionPD q_BI_meas);

        void reset();

        void setGains(const Eigen::Vector3d kp, const Eigen::Vector3d ki, const Eigen::Vector3d kd) {
            setKp(kp);
            setKi(ki);
            setKd(kd);
        }

        inline void setKp(const Eigen::Vector3d kp) { kp_.setValue(kp); }
        inline void setKi(const Eigen::Vector3d ki) { ki_.setValue(ki); }
        inline void setKd(const Eigen::Vector3d kd) { kd_.setValue(kd); }
        inline void setMaxEffort(const Eigen::Vector3d eff) { maxEffort_.setValue(eff); }

        inline Eigen::Vector3d getKp() const { return kp_.getValue(); }
        inline Eigen::Vector3d getKi() const { return ki_.getValue(); }
        inline Eigen::Vector3d getKd() const { return kd_.getValue(); }
        inline Eigen::Vector3d getMaxEffort() const { return maxEffort_.getValue(); }

        bool addParametersToHandler(const std::string &pidName);

    protected:
        Eigen::Vector3d integral_;
        kindr::RotationQuaternionPD q_BI_prev_;
        parameter_handler::Parameter <Eigen::Vector3d> kp_;
        parameter_handler::Parameter <Eigen::Vector3d> ki_;
        parameter_handler::Parameter <Eigen::Vector3d> kd_;
        parameter_handler::Parameter <Eigen::Vector3d> maxEffort_;
    };

} // namespace robot_utils