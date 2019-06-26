



%% Mainbody velocities
if (plotMainbodyVel)

    named_figure('main body linear velocity')
    subplot(3,1,1)
    plot(time, data(:,idx_rm_q_linearVelocityBaseInWorldFrame_x), 'b')
    plot(time, data(:,idx_loco_torso_desiredLinearVelocityBaseInControlFrame_x), 'r')
    
    if (plotModeSwitches)
        vline(time(idx_LF_m),'b')
        vline(time(idx_RF_m),'c')
        vline(time(idx_LH_m),'r')
        vline(time(idx_RH_m),'m')
    end
    grid on
    xlabel('time [s]')
    ylabel('x [m/s]')
    title('linear velocity of torso expressed in world frame')
    subplot(3,1,2)
    plot(time, data(:,idx_rm_q_linearVelocityBaseInWorldFrame_y))
    grid on
    xlabel('time [s]')
    ylabel('y [m/s]')
    subplot(3,1,3)
    plot(time, data(:,idx_rm_q_linearVelocityBaseInWorldFrame_z))
    grid on
    xlabel('time [s]')
    ylabel('z [m/s]')
    
    
    named_figure('main body reference linear velocity in control frame')
    subplot(3,1,1)
    plot(time, data(:,idx_loco_torso_desiredLinearVelocityBaseInControlFrame_x), 'r')
    grid on
    xlabel('time [s]')
    ylabel('x [m/s]')
    title('linear velocity of torso expressed in control frame')
    subplot(3,1,2)
    plot(time, data(:,idx_loco_torso_desiredLinearVelocityBaseInControlFrame_y), 'r')
    grid on
    xlabel('time [s]')
    ylabel('y [m/s]')
    subplot(3,1,3)
    plot(time, data(:,idx_loco_torso_desiredLinearVelocityBaseInControlFrame_z), 'r')
    grid on
    xlabel('time [s]')
    ylabel('z [m/s]')
    
    
    
    named_figure('main body angular velocity expressed in base frame')
    subplot(3,1,1)
    plot(time, data(:,idx_rm_q_qAngVel_x))
    grid on
    xlabel('time [s]')
    ylabel('x [rad/s]')
    subplot(3,1,2)
    plot(time, data(:,idx_rm_q_qAngVel_y))
    grid on
    xlabel('time [s]')
    ylabel('y [rad/s]')
    subplot(3,1,3)
    plot(time, data(:,idx_rm_q_qAngVel_z))
    grid on
    xlabel('time [s]')
    ylabel('z [rad/s]')
end


%% Measurred joint positions
if (plotJointPositionsMeas)
    
    named_figure('Joints LF'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_log_LF_HAA_th))
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_log_LF_HFE_th))
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_log_LF_KFE_th))
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    named_figure('Joints RF'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_log_RF_HAA_th))
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_log_RF_HFE_th))
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_log_RF_KFE_th))
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    named_figure('Joints LH'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_log_LH_HAA_th))
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_log_LH_HFE_th))
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_log_LH_KFE_th))
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')


    named_figure('Joints RH'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_log_RH_HAA_th))
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_log_RH_HFE_th))
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_log_RH_KFE_th))
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

end
%% measured and desired joint positions
if (plotJointPositionsMeasAndDes)
    named_figure('measured and desired joint positions'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_log_LF_HAA_des_th),'r.-')
    plot(time, data(:,idx_log_LF_HAA_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_des_th),'r.-')
    plot(time, data(:,idx_log_LF_HFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_des_th),'r.-')
    plot(time, data(:,idx_log_LF_KFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_des_th),'r.-')
    plot(time, data(:,idx_log_RF_HAA_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_log_RF_HFE_des_th),'r.-')
    plot(time, data(:,idx_log_RF_HFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_des_th),'r.-')
    plot(time, data(:,idx_log_RF_KFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_des_th),'r.-')
    plot(time, data(:,idx_log_LH_HAA_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_des_th),'r.-')
    plot(time, data(:,idx_log_LH_HFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_des_th),'r.-')
    plot(time, data(:,idx_log_LH_KFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_des_th),'r.-')
    plot(time, data(:,idx_log_RH_HAA_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_des_th),'r.-')
    plot(time, data(:,idx_log_RH_HFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_des_th),'r.-')
    plot(time, data(:,idx_log_RH_KFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

end






%% measured joint velocities
if (plotJointVelocitiesMeas)
    named_figure('measured joint velocities'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_log_LF_HAA_thd),'b.-')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_thd),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_thd),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_thd),'b.-')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_log_RF_HFE_thd),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_thd),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_thd),'b.-')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_thd),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_thd),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_thd),'b.-')
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_thd),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_thd),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

end

%% measured and desired joint velocities
if (plotJointVelocitiesMeasAndDes)
    named_figure('measured and desired joint velocities'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_rm_q_dqLFHAA),'b.-')
    plot(time, data(:,idx_log_LF_HAA_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_thd),'b.-')
    plot(time, data(:,idx_log_LF_HFE_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_thd),'b.-')
    plot(time, data(:,idx_log_LF_KFE_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_thd),'b.-')
    plot(time, data(:,idx_log_RF_HAA_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_log_RF_HFE_thd),'b.-')
    plot(time, data(:,idx_log_RF_HFE_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_thd),'b.-')
    plot(time, data(:,idx_log_RF_KFE_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_thd),'b.-')
    plot(time, data(:,idx_log_LH_HAA_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_thd),'b.-')
    plot(time, data(:,idx_log_LH_HFE_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_thd),'b.-')
    plot(time, data(:,idx_log_LH_KFE_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_thd),'b.-')
    plot(time, data(:,idx_log_RH_HAA_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_thd),'b.-')
    plot(time, data(:,idx_log_RH_HFE_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_rm_q_dqRHKFE),'b.-')
    plot(time, data(:,idx_log_RH_KFE_des_thd),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

end
%% Joint torques

if (plotJointTorques)
    named_figure('joint torques'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_log_LF_HAA_load),'b.-')
    plot(time, data(:,idx_log_LF_HAA_uff), 'r.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_load),'b.-')
    plot(time, data(:,idx_log_LF_HFE_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_load),'b.-')
    plot(time, data(:,idx_log_LF_KFE_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_load),'b.-')
    plot(time, data(:,idx_log_RF_HAA_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_log_RF_HFE_load),'b.-')
    plot(time, data(:,idx_log_RF_HFE_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_load),'b.-')
    plot(time, data(:,idx_log_RF_KFE_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_load),'b.-')
    plot(time, data(:,idx_log_LH_HAA_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_load),'b.-')
    plot(time, data(:,idx_log_LH_HFE_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_load),'b.-')
    plot(time, data(:,idx_log_LH_KFE_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_load),'b.-')
    plot(time, data(:,idx_log_RH_HAA_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_load),'b.-')
    plot(time, data(:,idx_log_RH_HFE_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_load),'b.-')
    plot(time, data(:,idx_log_RH_KFE_uff),'r.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

end
%% Mode
if (plotLocoControlModes)
    named_figure('loco control modes'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_desControlMode_HAA),'b')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_desControlMode_HFE),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_desControlMode_KFE),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_desControlMode_HAA),'b')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_desControlMode_HFE),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_desControlMode_KFE),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_desControlMode_HAA),'b')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_desControlMode_HFE),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_desControlMode_KFE),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_desControlMode_HAA),'b')
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_desControlMode_HFE),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_desControlMode_KFE),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')
end


%% Measured motor velocities
if (plotMotorVelocitiesMeas)
    named_figure('measured motor velocities'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_log_LF_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_log_RF_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]/s')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

end


%% Measured motor velocities measured and desired
if (plotMotorVelocitiesMeas)
    named_figure('measured motor velocities meas and es'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_log_LF_HAA_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_LF_HAA_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_LF_HFE_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_LF_KFE_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_RF_HAA_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_log_RF_HFE_MOTOR_VEL),'b')
    plot(time, data(:,idx_RF_HFE_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_RF_KFE_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_LH_HAA_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]/s')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_LH_HFE_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_LH_KFE_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_RH_HAA_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_RH_HFE_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_MOTOR_VEL),'b')
    plot(time, data(:,idx_log_RH_KFE_MOTOR_VEL_DES),'r')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

end
%% Measured motor current
if (plotMotorCurrent)
    named_figure('measured motor current'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_log_LF_HAA_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_log_RF_HFE_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]/s')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_MOTOR_CURRENT),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

end


%%
if (plotDeflections)
    tol = 0.02;
    tolHigh = 0.07;
    
    named_figure('Deflections'), clf
    grid on
    subplot(1,4,1)
    hold on
    plot(time, data(:,idx_log_LF_KFE_DEFL),'b')
    hline(tol,'r')
    hline(tolHigh,'r')
    plot(time, data(:,idx_log_LF_CONTACT_FLAG)*tolHigh,'c-')
    grid on
    title('LF')
    xlabel('time [s]')
    ylabel('deflection [rad]')
    
    subplot(1,4,2)
    hold on
    plot(time, data(:,idx_log_RF_KFE_DEFL),'b')
    hline(tol,'r')
    hline(tolHigh,'r')
    plot(time, data(:,idx_log_RF_CONTACT_FLAG)*tolHigh,'c-')
    grid on
    title('RF')
    xlabel('time [s]')
    ylabel('deflection [rad]')
    
    subplot(1,4,3)
    hold on
    plot(time, -data(:,idx_log_LH_KFE_DEFL),'b')
    hline(tol,'r')
    hline(tolHigh,'r')
    plot(time, data(:,idx_log_LH_CONTACT_FLAG)*tolHigh,'c-')
    grid on
    title('LH')
    xlabel('time [s]')
    ylabel('deflection [rad]')
    
    subplot(1,4,4)
    hold on
    plot(time, -data(:,idx_log_RH_KFE_DEFL),'b')
    grid on
    hline(tol,'r')
    hline(tolHigh,'r')
    plot(time, data(:,idx_log_RH_CONTACT_FLAG)*tolHigh,'c-')
    title('RH')
    xlabel('time [s]')
    ylabel('deflection [rad]')
end

%%
if (plotPhases)
   dt = 0;
  named_figure('phase'), clf
    subplot(4,1,1)
    hold on
    plot(time, data(:,idx_LFStanceP),'b')
    plot(time, data(:,idx_LFSwingP),'r')
    plot(time, data(:,idx_LFGrounded),'k')
    plot(time, data(:,idx_LF_CONTACT_FLAG),'m--')
    plot(time, LF_m_scaled,'c--')
    plot(time, data(:,idx_LFStMode),'b:')
    %ylim([0 1.1])
    vline(dt,'c')
    title('LF')
    grid on
    legend('stance phase','swing phase','grounded flag', 'contact flag','control mode','stance mode')
    subplot(4,1,2)
    hold on
    plot(time, data(:,idx_RFStanceP),'b')
    plot(time, data(:,idx_RFSwingP),'r')
    plot(time, data(:,idx_RFGrounded),'k')
    plot(time, data(:,idx_RF_CONTACT_FLAG),'m--')
    plot(time, RF_m_scaled,'c--')
    plot(time, data(:,idx_RFStMode),'b:')
    vline(dt,'c')
    title('RF')
    grid on
    subplot(4,1,3)
    hold on
    plot(time, data(:,idx_LHStanceP),'b')
    plot(time, data(:,idx_LHSwingP),'r')
    plot(time, data(:,idx_LHGrounded),'k')
    plot(time, data(:,idx_LH_CONTACT_FLAG),'m--')
    plot(time, LH_m_scaled,'c--')
    plot(time, data(:,idx_LHStMode),'b:')
    vline(dt,'c')
    grid on
    title('LH')
    subplot(4,1,4)
    hold on
    plot(time, data(:,idx_RHStanceP),'b')
    plot(time, data(:,idx_RHSwingP),'r')
    plot(time, data(:,idx_RHGrounded),'k')
    plot(time, data(:,idx_RH_CONTACT_FLAG),'m--')
    plot(time, RH_m_scaled,'c--')
    plot(time, data(:,idx_RHStMode),'b:')
    vline(dt,'c')
    grid on
    title('RH')
    
end
%%  Plot APS
if (plotAPS)
    
    % color codes for legs
    colors = [214 1 39; 231 165 0; 0 96 40; 49 180 48]/255;
    
    stance = zeros(length(time),4);
    stance(:,1) = data(:,idx_LFStanceP)~=0;
    stance(:,2) = data(:,idx_RFStanceP)~=0;
    stance(:,3) = data(:,idx_LHStanceP)~=0;
    stance(:,4) = data(:,idx_RHStanceP)~=0;

    yLines = [4 3 2 1];
    hFig = figure(2);clf;
    legNames = {'RH', 'LH', 'RF', 'LF'};
    axes1 = axes('Parent',hFig,...
        'YTick',yLines(end:-1:1), ...
        'YTickLabel',legNames, ...
        'XGrid','off', ...
        'TickLength', [0,0]);

    box(axes1,'on');
    hold(axes1,'all');

    yStart = (yLines(end)+(yLines(end)-yLines(end-1))/2);
    yEnd = (yLines(1)-(yLines(end)-yLines(end-1))/2);
    %line([stridePhase; stridePhase], [yStart*ones(1,length(stridePhase)); yEnd*ones(1,length(stridePhase))], 'Color', 'k', 'LineStyle', '--')
    %nAPS = length(apsEnd)-1;
    %line([apsStart(1:1:nAPS); apsEnd(2:1:end)], [yEnd*ones(1,nAPS); yStart*ones(1,nAPS)], 'Color', 'k', 'LineStyle', '-')

    % plot(time, yLines(1)*stance(:,1), 'rs')
    % plot(time, yLines(2)*stance(:,2), 'bs')
    % plot(time, yLines(3)*stance(:,3), 'ms')
    % plot(time, yLines(4)*stance(:,4), 'cs')

    ylim([yStart yEnd])
    xlabel('time [s]')
    title('Gait diagram')

    hSpace = 0.05;
    hBar = (yLines(1)-yLines(2))/2-hSpace;
    hLow = yLines-hBar;
    hHigh = yLines+hBar;
    hLine = hLow+hSpace/2;
    hLine = [hLine hHigh(end)-hSpace/2];

    iLeg = 4;

    for iLeg=1:1:4
        stancePhase = stance(:,iLeg);
        idxTD = find(diff(stancePhase)==1);
        idxLO = find(diff(stancePhase)==-1);
        if (length(idxTD) > length(idxLO))
           idxLO = [idxLO; length(stancePhase)];
        elseif (length(idxTD) < length(idxLO))
            idxTD = [1; idxTD];
        else
            if (stancePhase(1) == 1 && idxTD(1)~=1 )
                 idxTD = [1; idxTD];
                 idxLO = [idxLO; length(stancePhase)];
            end
        end

        for k=1:length(idxTD)
            areaX = [time(idxLO(k)) time(idxTD(k)) ];
            areaY = hHigh(iLeg)*[1 1];
            h = area(areaX, areaY);
            set(h(1),'FaceColor',colors(iLeg, :))
            set(h(1),'edgecolor','none')
            set(h(1),'BaseValue',hLow(iLeg))
        end
    end

end
%% Contact Flags
if (plotContactFlags)
    named_figure('contact flags'), clf
    subplot(4,1,1)
    hold on
    plot(time, data(:,idx_log_LF_CONTACT_FLAG),'b')
    title('LF')
    grid on
    subplot(4,1,2)
    hold on
    plot(time, data(:,idx_log_RF_CONTACT_FLAG),'b')
    title('RF')
    grid on
    subplot(4,1,3)
    hold on
    plot(time, data(:,idx_log_LH_CONTACT_FLAG),'b')
    grid on
    title('LH')
    subplot(4,1,4)
    hold on
    plot(time, data(:,idx_log_RH_CONTACT_FLAG),'b')
    grid on
    title('RH')
end
    
%% motor power
if (plotMotorPower)
    named_figure('motor power'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_log_LF_HAA_load).*data(:,idx_log_LF_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_load).*data(:,idx_log_LF_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_load).*data(:,idx_log_LF_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_load).*data(:,idx_log_RF_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_load).*data(:,idx_RF_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_load).*data(:,idx_log_RF_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_load).*data(:,idx_log_LH_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_load).*data(:,idx_log_LH_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_load).*data(:,idx_log_LH_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_load).*data(:,idx_log_RH_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_load).*data(:,idx_log_RH_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_load).*data(:,idx_log_RH_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')
end
%%
if (plotJointPower)
    
    LF_HAA_power = data(:,idx_log_LF_HAA_load).*data(:,idx_log_LF_HAA_thd);
    LF_HFE_power = data(:,idx_log_LF_HFE_load).*data(:,idx_log_LF_HFE_thd);
    LF_KFE_power = data(:,idx_log_LF_KFE_load).*data(:,idx_log_LF_KFE_thd);
    RF_HAA_power = data(:,idx_log_RF_HAA_load).*data(:,idx_log_RF_HAA_thd);
    RF_HFE_power = data(:,idx_log_RF_HFE_load).*data(:,idx_log_RF_HFE_thd);
    RF_KFE_power = data(:,idx_log_RF_KFE_load).*data(:,idx_log_RF_KFE_thd);
    LH_HAA_power = data(:,idx_log_LH_HAA_load).*data(:,idx_log_LH_HAA_thd);
    LH_HFE_power = data(:,idx_log_LH_HFE_load).*data(:,idx_log_LH_HFE_thd);
    LH_KFE_power = data(:,idx_log_LH_KFE_load).*data(:,idx_log_LH_KFE_thd);
    RH_HAA_power = data(:,idx_log_RH_HAA_load).*data(:,idx_log_RH_HAA_thd);
    RH_HFE_power = data(:,idx_log_RH_HFE_load).*data(:,idx_log_RH_HFE_thd);
    RH_KFE_power = data(:,idx_log_RH_KFE_load).*data(:,idx_log_RH_KFE_thd);
    
    power = [LF_HAA_power; LF_HFE_power; LF_KFE_power; RF_HAA_power; RF_HFE_power;  RF_KFE_power; LH_HAA_power; LH_HFE_power; LH_KFE_power; RH_HAA_power; RH_HFE_power; RH_KFE_power];
    meanPower = mean(power);
    
    named_figure('joint power'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, LF_HAA_power,'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,5)
    hold on
    plot(time, LF_HFE_power,'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,9)
    hold on
    plot(time, LF_KFE_power,'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_load).*data(:,idx_log_RF_HAA_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_log_RF_HFE_load).*data(:,idx_log_RF_HFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_load).*data(:,idx_log_RF_KFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_load).*data(:,idx_log_LH_HAA_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_load).*data(:,idx_log_LH_HFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_load).*data(:,idx_log_LH_KFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_load).*data(:,idx_log_RH_HAA_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_load).*data(:,idx_log_RH_HFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_load).*data(:,idx_log_RH_KFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')
end
%% 3D plot
   
if (plot3DPose)
    
    tStart =10.8;
    tEnd =  tStart+0.2;
    
    
    if (~exist('sys', 'var'))
        load starlETHSystem
    end

    plotParams.color.swingLeg = 'b';
    plotParams.color.stanceLeg = 'r';

    plotParams.color.swingFoot = 'b';
    plotParams.color.stanceFoot = 'r';

    plotParams.color.supportPolygon = [0 0.5 0];
    plotParams.drawSupportPolygon = 1;


    [timeRange, dataRange] = getDataInTimeRange(time, data, tStart, tEnd);

    named_figure('myrobot'),clf
    hold on
    idx=1:50:length(timeRange);
    colors = varycolor(length(idx));
    q = getQFromData(idx, dataRange);
    contactFlags = getContactFlagsFromData(idx, dataRange);
    k = 1;
    for i=idx
        plotParams.color.mainBody = colors(k,:);
        plotParams.color.swingLeg = colors(k,:);
        plotParams.color.stanceLeg = colors(k,:);
        plotParams.color.supportPolygon = colors(k,:);
        plotStarlETH(q(k,:), contactFlags(k,:), plotParams);
        k = k + 1;
    end

end



%%

if (plotFootholdEstimates)
    named_figure('Estimated Foothold Positions'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_ocEKF_p11),'b')
    plot(time, data(:,idx_ocEKF_stdl_p11),'g:')
    plot(time, 2*data(:,idx_ocEKF_p11)-data(:,idx_ocEKF_stdl_p11),'g:')
    title('Coordinates of foothold 1')
    grid on
    xlabel('time [s]')
    ylabel('x [m]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_ocEKF_p12),'b')
    grid on
    xlabel('time [s]')
    ylabel('y [m]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_ocEKF_p13),'b')
    grid on
    xlabel('time [s]')
    ylabel('z [m]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_ocEKF_p21),'b')
    title('Coordinates of foothold 2')
    grid on
    xlabel('time [s]')
    ylabel('x [m]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_ocEKF_p22),'b')
    grid on
    xlabel('time [s]')
    ylabel('y [m]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_ocEKF_p23),'b')
    grid on
    xlabel('time [s]')
    ylabel('z [m]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_ocEKF_p31),'b')
    title('Coordinates of foothold 3')
    grid on
    xlabel('time [s]')
    ylabel('x [m]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_ocEKF_p32),'b')
    grid on
    xlabel('time [s]')
    ylabel('y [m]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_ocEKF_p33),'b')
    grid on
    xlabel('time [s]')
    ylabel('z [m]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_ocEKF_p41),'b')
    title('Coordinates of foothold 4')
    grid on
    xlabel('time [s]')
    ylabel('x [m]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_ocEKF_p42),'b')
    grid on
    xlabel('time [s]')
    ylabel('y [m]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_ocEKF_p43),'b')
    grid on
    xlabel('time [s]')
    ylabel('z [m]')

end

%%
if (plotMainbodyPositionFromMocap)
    named_figure('main body position mocap'), clf
    
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_ROS_MOCAP_T_X),'b');
    grid on
    xlabel('time [s]')
    ylabel('x')

    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_ROS_MOCAP_T_Y),'b');
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_ROS_MOCAP_T_Z),'b');
    grid on
    xlabel('time [s]')
    ylabel('z')
end

%%
if (plotLocoVMCandCFD) 
    named_figure('Virtual Net Force'), clf
    
    subplot(4,1,1)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_isSupportLeg)+data(:,idx_loco_rightForeLeg_isSupportLeg)+data(:,idx_loco_leftHindLeg_isSupportLeg)+data(:,idx_loco_rightHindLeg_isSupportLeg),'r');
    plot(time, data(:,idx_loco_leftForeLeg_isGrounded)+data(:,idx_loco_rightForeLeg_isGrounded)+data(:,idx_loco_leftHindLeg_isGrounded)+data(:,idx_loco_rightHindLeg_isGrounded),'b');
    legend('# support legs', '# contacts')
    grid on
    xlabel('time [s]')
    ylabel('-')
    title('# contacts')
    
    subplot(4,1,2)
    hold on
    plot(time, data(:,idx_loco_vmc_desVirtualForceInBaseFrame_x),'r');
    plot(time, data(:,idx_loco_cfd_distVirtualForceInBaseFrame_x),'b');
    grid on
    xlabel('time [s]')
    ylabel('x')

    legend('desired', 'distributed')
    title('Virtual Net Force')
    
    subplot(4,1,3)
    hold on
    plot(time, data(:,idx_loco_vmc_desVirtualForceInBaseFrame_y),'r');
    plot(time, data(:,idx_loco_cfd_distVirtualForceInBaseFrame_y),'b');
    grid on
    xlabel('time [s]')
    ylabel('y')
    
    subplot(4,1,4)
    hold on
    plot(time, data(:,idx_loco_vmc_desVirtualForceInBaseFrame_z),'r');
    plot(time, data(:,idx_loco_cfd_distVirtualForceInBaseFrame_z),'b');
    grid on
    xlabel('time [s]')
    ylabel('z') 
    
    
    named_figure('Virtual Net Torque'), clf
    
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_loco_vmc_desVirtualTorqueInBaseFrame_x),'r');
    plot(time, data(:,idx_loco_cfd_distVirtualTorqueInBaseFrame_x),'b');
    grid on
    xlabel('time [s]')
    ylabel('x')

    legend('desired', 'distributed')
    title('Virtual Net Torque')
    
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_loco_vmc_desVirtualTorqueInBaseFrame_y),'r');
    plot(time, data(:,idx_loco_cfd_distVirtualTorqueInBaseFrame_y),'b');
    grid on
    xlabel('time [s]')
    ylabel('y')
    
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_loco_vmc_desVirtualTorqueInBaseFrame_z),'r');
    plot(time, data(:,idx_loco_cfd_distVirtualTorqueInBaseFrame_z),'b');
    grid on
    xlabel('time [s]')
    ylabel('z') 
end


%%
if (plotLocoCFD) 

    named_figure('Desired contact forces'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_x),'r')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_y),'r')
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_desiredContactForceAtFootInWorldFrame_z),'r')
    grid on
    xlabel('time [s]')
    ylabel('z')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_x),'r')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_y),'r')
    grid on
    xlabel('time [s]')
    ylabel('y]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_desiredContactForceAtFootInWorldFrame_z),'r')
    grid on
    xlabel('time [s]')
    ylabel('z')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_x),'r')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_y),'r')
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_desiredContactForceAtFootInWorldFrame_z),'r')
    grid on
    xlabel('time [s]')
    ylabel('z')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_x),'r')
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_y),'r')
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_desiredContactForceAtFootInWorldFrame_z),'r')
    grid on
    xlabel('time [s]')
    ylabel('z')
end


%%

if (plotLocoIsGrounded)
    named_figure('leg is grounded'), clf
    
    subplot(4,1,1)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_isGrounded),'b');
    grid on
    xlabel('time [s]')
    ylabel('LF')

    title('grounded')
    
    subplot(4,1,2)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_isGrounded),'b');
    grid on
    xlabel('time [s]')
    ylabel('RF')
    
        
    subplot(4,1,3)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_isGrounded),'b');
    grid on
    xlabel('time [s]')
    ylabel('LH') 
    
    subplot(4,1,4)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_isGrounded),'b');
    grid on
    xlabel('time [s]')
    ylabel('RH') 
    
end

%% Loco Torso
if (plotLocoTorso)

    named_figure('torso position expressed in control frame'), clf
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_loco_torso_measPositionControlToBaseInControlFrame_x), 'b')
    plot(time, data(:,idx_loco_torso_desiredPositionControlToBaseInControlFrame_x), 'r')
    grid on
    xlabel('time [s]')
    ylabel('x [rad/s]')
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_loco_torso_measPositionControlToBaseInControlFrame_y), 'b')
    plot(time, data(:,idx_loco_torso_desiredPositionControlToBaseInControlFrame_y), 'r')
    grid on
    xlabel('time [s]')
    ylabel('y [rad/s]')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_loco_torso_measPositionControlToBaseInControlFrame_z), 'b')
    plot(time, data(:,idx_loco_torso_desiredPositionControlToBaseInControlFrame_z), 'r')
    grid on
    xlabel('time [s]')
    ylabel('z [rad/s]')
    
    %%
    named_figure('torso position error')
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_loco_torso_positionErrorInControlFrame_x), 'b')
    grid on
    xlabel('time [s]')
    ylabel('x [m]')
    title('position error in control frame')
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_loco_torso_positionErrorInControlFrame_y),'b')
    grid on
    xlabel('time [s]')
    ylabel('y [m]')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_loco_torso_positionErrorInControlFrame_z), 'b')
    grid on
    xlabel('time [s]')
    ylabel('z [m]')
    
    %%
    named_figure('toroso linear velocity')
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_loco_torso_measLinearVelocityBaseInControlFrame_x), 'b')
    plot(time, data(:,idx_loco_torso_desiredLinearVelocityBaseInControlFrame_x), 'r')
    grid on
    xlabel('time [s]')
    ylabel('x [m/s]')
    title('linear velocity of torso expressed in control frame')
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_loco_torso_measLinearVelocityBaseInControlFrame_y),'b')
    plot(time, data(:,idx_loco_torso_desiredLinearVelocityBaseInControlFrame_y), 'r')
    grid on
    xlabel('time [s]')
    ylabel('y [m/s]')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_loco_torso_measLinearVelocityBaseInControlFrame_z), 'b')
    plot(time, data(:,idx_loco_torso_desiredLinearVelocityBaseInControlFrame_z), 'r')
    grid on
    xlabel('time [s]')
    ylabel('z [m/s]')
    

   
    %%
    
    named_figure('linear velocity torso error'), clf
    subplot(3,1,1)
    plot(time, data(:,idx_loco_torso_linearVelocityErrorInControlFrame_x), 'b')
    grid on
    xlabel('time [s]')
    ylabel('x [m/s]')
    title('linear velocity error in control frame')
    subplot(3,1,2)
    plot(time, data(:,idx_loco_torso_linearVelocityErrorInControlFrame_y), 'b')
    grid on
    xlabel('time [s]')
    ylabel('y [m/s]')
    subplot(3,1,3)
    plot(time, data(:,idx_loco_torso_linearVelocityErrorInControlFrame_z), 'b')
    grid on
    xlabel('time [s]')
    ylabel('z [m/s]')
    
    
    %%
    named_figure('torso angular velocity expressed in control frame'), clf
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_loco_torso_measAngularVelocityBaseInControlFrame_x), 'b')
    plot(time, data(:,idx_loco_torso_desiredAngularVelocityBaseInControlFrame_x), 'r')
    grid on
    xlabel('time [s]')
    ylabel('x [rad/s]')
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_loco_torso_measAngularVelocityBaseInControlFrame_y), 'b')
    plot(time, data(:,idx_loco_torso_desiredAngularVelocityBaseInControlFrame_y), 'r')
    grid on
    xlabel('time [s]')
    ylabel('y [rad/s]')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_loco_torso_measAngularVelocityBaseInControlFrame_z), 'b')
    plot(time, data(:,idx_loco_torso_desiredAngularVelocityBaseInControlFrame_z), 'r')
    grid on
    xlabel('time [s]')
    ylabel('z [rad/s]')
end


%%
if (plotLocoLegState)
    named_figure('leg state'), clf
    
    subplot(4,1,1)
    hold on
    plot(time, data(:,idx_loco_leftForeLeg_isGrounded),'b');
    plot(time, data(:,idx_loco_leftForeLeg_shouldBeGrounded),'r');
    plot(time, data(:,idx_loco_leftForeLeg_isSupportLeg),'k--');
    plot(time, data(:,idx_loco_leftForeLeg_desControlMode_HAA)-2,'g:');
    plot(time, data(:,idx_loco_leftForeLeg_limbState),'m-');
    grid on
    xlabel('time [s]')
    ylabel('LF')
    
    legend('isGrounded', 'shouldBeGrounded', 'isSupportLeg','desControlModeHAA')
    title('Leg state')
    
    subplot(4,1,2)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_isGrounded),'b');
    plot(time, data(:,idx_loco_rightForeLeg_shouldBeGrounded),'r');
    plot(time, data(:,idx_loco_rightForeLeg_isSupportLeg),'k--');
    plot(time, data(:,idx_loco_rightForeLeg_desControlMode_HAA)-2,'g:');
    plot(time, data(:,idx_loco_rightForeLeg_limbState),'m-');
    grid on
    xlabel('time [s]')
    ylabel('RF')
    
        
    subplot(4,1,3)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_isGrounded),'b');
    plot(time, data(:,idx_loco_leftHindLeg_shouldBeGrounded),'r');
    plot(time, data(:,idx_loco_leftHindLeg_isSupportLeg),'k--');
    plot(time, data(:,idx_loco_leftHindLeg_desControlMode_HAA)-2,'g:');
    plot(time, data(:,idx_loco_leftHindLeg_limbState),'m-');
    grid on
    xlabel('time [s]')
    ylabel('LH') 
    
    subplot(4,1,4)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_isGrounded),'b');
    plot(time, data(:,idx_loco_rightHindLeg_shouldBeGrounded),'r');
    plot(time, data(:,idx_loco_rightHindLeg_isSupportLeg),'k--');
    plot(time, data(:,idx_loco_rightHindLeg_desControlMode_HAA)-2,'g:');
    plot(time, data(:,idx_loco_rightHindLeg_limbState),'m-');
    grid on
    xlabel('time [s]')
    ylabel('RH') 
    
end


%%
if (plotOptoforces) 
    contactForceThreshold = 600;
    named_figure('Contact forces from optoforce sensors'), clf
    grid on
    subplot(4,4,1)
    hold on
    plot(time, sqrt((data(:,idx_rm_contacts_LF_FOOT_forceInWorlFrame_x).^2+data(:,idx_rm_contacts_LF_FOOT_forceInWorlFrame_y).^2+data(:,idx_rm_contacts_LF_FOOT_forceInWorlFrame_z).^2)),'r')
    hline(contactForceThreshold,'-')
    x = data(2:end,idx_rm_contacts_LF_FOOT_state);
    y = data(1:end-1,idx_rm_contacts_LF_FOOT_state);
    idx_contactSwitchLf = find((x(1:end)~=y(1:end)))+1;
    if (~isempty(idx_contactSwitchLf))
        vline(data(idx_contactSwitchLf),'k-')
    end
    xlim([tStart time(end)])
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('norm')  
    subplot(4,4,5)
    hold on
    plot(time, data(:,idx_rm_contacts_LF_FOOT_forceInWorlFrame_x),'r')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(4,4,9)
    hold on
    plot(time, data(:,idx_rm_contacts_LF_FOOT_forceInWorlFrame_y),'r')
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(4,4,13)
    hold on
    plot(time, data(:,idx_rm_contacts_LF_FOOT_forceInWorlFrame_z),'r')
    grid on
    xlabel('time [s]')
    ylabel('z')

    
    
    subplot(4,4,2)
    hold on
    plot(time, sqrt((data(:,idx_rm_contacts_RF_FOOT_forceInWorlFrame_x).^2+data(:,idx_rm_contacts_RF_FOOT_forceInWorlFrame_y).^2+data(:,idx_rm_contacts_RF_FOOT_forceInWorlFrame_z).^2)),'r')
    hline(contactForceThreshold,'-')
    x = data(2:end,idx_rm_contacts_RF_FOOT_state);
    y = data(1:end-1,idx_rm_contacts_RF_FOOT_state);
    idx_contactSwitchRf = find((x(1:end)~=y(1:end)))+1;
    if (~isempty(idx_contactSwitchRf))
        vline(data(idx_contactSwitchRf),'k-')
    end
    xlim([tStart time(end)])
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('norm')   
    subplot(4,4,6)
    hold on
    plot(time, data(:,idx_rm_contacts_RF_FOOT_forceInWorlFrame_x),'r')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(4,4,10)
    hold on
    plot(time, data(:,idx_rm_contacts_RF_FOOT_forceInWorlFrame_y),'r')
    grid on
    xlabel('time [s]')
    ylabel('y]')
    subplot(4,4,14)
    hold on
    plot(time, data(:,idx_rm_contacts_RF_FOOT_forceInWorlFrame_z),'r')
    grid on
    xlabel('time [s]')
    ylabel('z')

    subplot(4,4,3)
    hold on
    plot(time, sqrt((data(:,idx_rm_contacts_LH_FOOT_forceInWorlFrame_x).^2+data(:,idx_rm_contacts_LH_FOOT_forceInWorlFrame_y).^2+data(:,idx_rm_contacts_LH_FOOT_forceInWorlFrame_z).^2)),'r')
    hline(contactForceThreshold,'-')
    x = data(2:end,idx_rm_contacts_LH_FOOT_state);
    y = data(1:end-1,idx_rm_contacts_LH_FOOT_state);
    idx_contactSwitchLh = find((x(1:end)~=y(1:end)))+1;
    if (~isempty(idx_contactSwitchLh))
        vline(data(idx_contactSwitchLh),'k-')
    end
    xlim([tStart time(end)])
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('norm')   
    subplot(4,4,7)
    hold on
    plot(time, data(:,idx_rm_contacts_LH_FOOT_forceInWorlFrame_x),'r')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(4,4,11)
    hold on
    plot(time, data(:,idx_rm_contacts_LH_FOOT_forceInWorlFrame_y),'r')
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(4,4,15)
    hold on
    plot(time, data(:,idx_rm_contacts_LH_FOOT_forceInWorlFrame_z),'r')
    grid on
    xlabel('time [s]')
    ylabel('z')

    subplot(4,4,4)
    hold on
    plot(time, sqrt((data(:,idx_rm_contacts_RH_FOOT_forceInWorlFrame_x).^2+data(:,idx_rm_contacts_RH_FOOT_forceInWorlFrame_y).^2+data(:,idx_rm_contacts_RH_FOOT_forceInWorlFrame_z).^2)),'r')
    hline(contactForceThreshold,'-')
    x = data(2:end,idx_rm_contacts_RH_FOOT_state);
    y = data(1:end-1,idx_rm_contacts_RH_FOOT_state);
    idx_contactSwitchRh = find((x(1:end)~=y(1:end)))+1;
    if (~isempty(idx_contactSwitchRh))
        vline(data(idx_contactSwitchRh),'k-')
    end
    xlim([tStart time(end)])
    title('RH')
    grid on
    subplot(4,4,8)
    hold on
    plot(time, data(:,idx_rm_contacts_RH_FOOT_forceInWorlFrame_x),'r')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(4,4,12)
    hold on
    plot(time, data(:,idx_rm_contacts_RH_FOOT_forceInWorlFrame_y),'r')
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(4,4,16)
    hold on
    plot(time, data(:,idx_rm_contacts_RH_FOOT_forceInWorlFrame_z),'r')
    grid on
    xlabel('time [s]')
    ylabel('z')
end

%%
if (plotTerrain)
    named_figure('Terrain position and normal vector'), clf;
    
    subplot(1,2,1);
    title('Normal to plane in world frame');
    hold on;
    grid on;
    plot(time, data(:,idx_loco_terrainModel_normalInWorldFrame_x),'r');
    plot(time, data(:,idx_loco_terrainModel_normalInWorldFrame_y),'g');
    plot(time, data(:,idx_loco_terrainModel_normalInWorldFrame_z),'b');
    
    subplot(1,2,2);
    title('Plane position vector in world frame');
    hold on;
    grid on;
    plot(time, data(:,idx_loco_terrainModel_positionInWorldFrame_x),'r');
    plot(time, data(:,idx_loco_terrainModel_positionInWorldFrame_y),'g');
    plot(time, data(:,idx_loco_terrainModel_positionInWorldFrame_z),'b');
end


%%
if (plotDesiredFootPositions) 
    contactForceThreshold = 3000;
    named_figure('Desired foot positions'), clf
    
    % Left fore leg
    subplot(4,4,1);
    title('LF');
    
    grid on; hold on;
    
	x = data(2,idx_rm_contacts_LF_FOOT_state);
    y = data(1:end-1,idx_rm_contacts_LF_FOOT_state);
    idx_contactSwitchLf = find((x(1:end)~=y(1:end)))+1;
    if (~isempty(idx_contactSwitchLf))
        vline(data(idx_contactSwitchLf),'k-')
    end
    plot(time, data(:,idx_loco_leftForeLeg_currentState),'m-');
    plot(time, data(:,idx_loco_leftForeLeg_isGrounded),'r');
    plot(time, data(:,idx_rm_contacts_LF_FOOT_state), 'b.-');
    xlim([tStart time(end)]); xlabel('time [s]'); ylabel('norm');  
    
    subplot(4,4,5)
    hold on; grid on;
    plot(time, data(:,idx_loco_leftForeLeg_positionWorldToDesiredFootInWorldFrame_x),'r.-');
    plot(time, data(:,idx_loco_leftForeLeg_positionWorldToFootInWorldFrame_x),'b.-');
    xlim([tStart time(end)]); xlabel('time [s]'); ylabel('x');  
    
    subplot(4,4,9);
    hold on; grid on;
    plot(time, data(:,idx_loco_leftForeLeg_positionWorldToDesiredFootInWorldFrame_y),'r.-');
    plot(time, data(:,idx_loco_leftForeLeg_positionWorldToFootInWorldFrame_y),'b.-');
    xlim([tStart time(end)]); xlabel('time [s]'); ylabel('y');  
    
    subplot(4,4,13);
    hold on; grid on;
    plot(time, data(:,idx_loco_leftForeLeg_positionWorldToDesiredFootInWorldFrame_z),'r.-');
    plot(time, data(:,idx_loco_leftForeLeg_positionWorldToFootInWorldFrame_z),'b.-');
    xlim([tStart time(end)]); xlabel('time [s]'); ylabel('z');  
    
    % Right fore leg
    subplot(4,4,2)
    title('RF')
    hold on; grid on;
%     x = data(2:end,idx_rm_contacts_RF_FOOT_state);
%     y = data(1:end-1,idx_rm_contacts_RF_FOOT_state);
%     idx_contactSwitchRf = find((x(1:end)~=y(1:end)))+1;
%     if (~isempty(idx_contactSwitchRf))
%         vline(data(idx_contactSwitchRf),'k-')
%     end
%     plot(time, data(:,idx_loco_rightForeLeg_currentState),'m-');
%     plot(time, data(:,idx_loco_rightForeLeg_isGrounded),'r');
%     plot(time, data(:,idx_rm_contacts_RF_FOOT_state), 'b');
    xlim([tStart time(end)]);
    
    
    xlabel('time [s]'); ylabel('norm');
    subplot(4,4,6)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_positionWorldToDesiredFootInWorldFrame_x),'r')
    plot(time, data(:,idx_loco_rightForeLeg_positionWorldToFootInWorldFrame_x),'b')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(4,4,10)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_positionWorldToDesiredFootInWorldFrame_y),'r')
    plot(time, data(:,idx_loco_rightForeLeg_positionWorldToFootInWorldFrame_y),'b')
    grid on
    xlabel('time [s]')
    ylabel('y]')
    subplot(4,4,14)
    hold on
    plot(time, data(:,idx_loco_rightForeLeg_positionWorldToDesiredFootInWorldFrame_z),'r')
    plot(time, data(:,idx_loco_rightForeLeg_positionWorldToFootInWorldFrame_z),'b')
    grid on
    xlabel('time [s]')
    ylabel('z')

    subplot(4,4,3)
    hold on; grid on;
%     x = data(2:end,idx_rm_contacts_LH_FOOT_state);
%     y = data(1:end-1,idx_rm_contacts_LH_FOOT_state);
%     idx_contactSwitchLh = find((x(1:end)~=y(1:end)))+1;
%     if (~isempty(idx_contactSwitchLh))
%         vline(data(idx_contactSwitchLh),'k-')
%     end
%     plot(time, data(:,idx_loco_leftHindLeg_currentState),'m-');
%     plot(time, data(:,idx_loco_leftHindLeg_isGrounded),'r');
%     plot(time, data(:,idx_rm_contacts_LH_FOOT_state), 'b');
    xlim([tStart time(end)])
    title('LH')

    xlabel('time [s]')
    ylabel('norm')   
    
    subplot(4,4,7)
    hold on; grid on;
    plot(time, data(:,idx_loco_leftHindLeg_positionWorldToDesiredFootInWorldFrame_x),'r');
    plot(time, data(:,idx_loco_leftHindLeg_positionWorldToFootInWorldFrame_x),'b');
    xlabel('time [s]')
    ylabel('x')
    
    subplot(4,4,11)
    hold on; grid on;
    plot(time, data(:,idx_loco_leftHindLeg_positionWorldToDesiredFootInWorldFrame_y),'r');
    plot(time, data(:,idx_loco_leftHindLeg_positionWorldToFootInWorldFrame_y),'b');
    xlabel('time [s]')
    ylabel('y')
    
    subplot(4,4,15)
    hold on
    plot(time, data(:,idx_loco_leftHindLeg_positionWorldToDesiredFootInWorldFrame_z),'r')
    plot(time, data(:,idx_loco_leftHindLeg_positionWorldToFootInWorldFrame_z),'b')
    grid on
    xlabel('time [s]')
    ylabel('z')

    subplot(4,4,4)
    hold on
%     x = data(2:end,idx_rm_contacts_RH_FOOT_state);
%     y = data(1:end-1,idx_rm_contacts_RH_FOOT_state);
%     idx_contactSwitchRh = find((x(1:end)~=y(1:end)))+1;
%     if (~isempty(idx_contactSwitchRh))
%         vline(data(idx_contactSwitchRh),'k-')
%     end
%     plot(time, data(:,idx_loco_rightHindLeg_currentState),'m-');
%     plot(time, data(:,idx_loco_rightHindLeg_isGrounded),'r');
%     plot(time, data(:,idx_rm_contacts_RH_FOOT_state), 'b');
    xlim([tStart time(end)])
    title('RH')
    grid on
    subplot(4,4,8)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_positionWorldToDesiredFootInWorldFrame_x),'r')
    plot(time, data(:,idx_loco_rightHindLeg_positionWorldToFootInWorldFrame_x),'b')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(4,4,12)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_positionWorldToDesiredFootInWorldFrame_y),'r')
    plot(time, data(:,idx_loco_rightHindLeg_positionWorldToFootInWorldFrame_y),'b')
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(4,4,16)
    hold on
    plot(time, data(:,idx_loco_rightHindLeg_positionWorldToDesiredFootInWorldFrame_z),'r')
    plot(time, data(:,idx_loco_rightHindLeg_positionWorldToFootInWorldFrame_z),'b')
    grid on
    xlabel('time [s]')
    ylabel('z')
end

%%

% legend for y axis:
% 0: stance phase normal condition
% 1: swing phase normal condition
% 2: stance, but slipping
% 3: stance, but lost contact / not yet touch-down
% 4: swing, but late lift-off
% 5: late swing, but early touch-down
% 6: middle swing, but bumped into obstacle while swinging

named_figure('Limb coordinator state'), clf

subplot(4, 1, 1)
title('LF'); hold on; grid on;
plot(time, data(:,idx_loco_leftForeLeg_currentState),'r.-')
plot(time, data(:,idx_loco_leftForeLeg_stancePhase),'b--')
legend('current state', 'stance phase');
xlabel('t [s]');

subplot(4, 1, 2)
title('RF'); hold on; grid on;
plot(time, data(:,idx_loco_rightForeLeg_currentState),'r.-')
plot(time, data(:,idx_loco_rightForeLeg_stancePhase),'b--')
legend('current state', 'stance phase');
xlabel('t [s]');

subplot(4, 1, 3)
title('LH'); hold on; grid on;
plot(time, data(:,idx_loco_leftHindLeg_currentState),'r.-')
plot(time, data(:,idx_loco_leftHindLeg_stancePhase),'b--')
legend('current state', 'stance phase');
xlabel('t [s]');

subplot(4, 1, 4)
title('RH'); hold on; grid on;
plot(time, data(:,idx_loco_rightHindLeg_currentState),'r.-')
plot(time, data(:,idx_loco_rightHindLeg_stancePhase),'b.-')
legend('current state', 'stance phase');
xlabel('t [s]');

%%

named_figure('Limb coordinator state'), clf

subplot(4, 1, 1)
title('LF'); hold on; grid on; ylim([-0.2 1.2]);
plot(time, data(:,idx_rm_contacts_LF_FOOT_state));
xlabel('t [s]');

subplot(4, 1, 2)
title('RF'); hold on; grid on; ylim([-0.2 1.2]);
plot(time, data(:,idx_rm_contacts_RF_FOOT_state));
xlabel('t [s]');

subplot(4, 1, 3)
title('LH'); hold on; grid on; ylim([-0.2 1.2]);
plot(time, data(:,idx_rm_contacts_LH_FOOT_state));
xlabel('t [s]');

subplot(4, 1, 4)
title('RH'); hold on; grid on; ylim([-0.2 1.2]);
plot(time, data(:,idx_rm_contacts_RH_FOOT_state));
xlabel('t [s]');


%% 
named_figure('Contact forces'), clf

subplot(4, 1, 1);
title('LF'); hold on; grid on;
plot(time, 6000*data(:,idx_loco_leftForeLeg_stancePhase),'m');
plot(time, data(:,idx_loco_leftForeLeg_contactForceAtFootInWorldFrame_x),'r');
plot(time, data(:,idx_loco_leftForeLeg_contactForceAtFootInWorldFrame_y),'g');
plot(time, data(:,idx_loco_leftForeLeg_contactForceAtFootInWorldFrame_z),'b');
plot(time, sqrt((data(:,idx_loco_leftForeLeg_contactForceAtFootInWorldFrame_x).^2+data(:,idx_loco_leftForeLeg_contactForceAtFootInWorldFrame_y).^2+data(:,idx_loco_leftForeLeg_contactForceAtFootInWorldFrame_z).^2)),'k');
legend('fx', 'fy', 'fz', 'norm(f)');
xlabel('t [s]');

subplot(4, 1, 2);
title('RF'); hold on; grid on;
plot(time, 6000*data(:,idx_loco_rightForeLeg_stancePhase),'m');
plot(time, data(:,idx_loco_rightForeLeg_contactForceAtFootInWorldFrame_x),'r');
plot(time, data(:,idx_loco_rightForeLeg_contactForceAtFootInWorldFrame_y),'g');
plot(time, data(:,idx_loco_rightForeLeg_contactForceAtFootInWorldFrame_z),'b');
plot(time, sqrt((data(:,idx_loco_rightForeLeg_contactForceAtFootInWorldFrame_x).^2+data(:,idx_loco_rightForeLeg_contactForceAtFootInWorldFrame_y).^2+data(:,idx_loco_rightForeLeg_contactForceAtFootInWorldFrame_z).^2)),'k');
legend('fx', 'fy', 'fz', 'norm(f)');
xlabel('t [s]');

subplot(4, 1, 3);
title('LH'); hold on; grid on;
plot(time, 6000*data(:,idx_loco_leftHindLeg_stancePhase),'m');
plot(time, data(:,idx_loco_leftHindLeg_contactForceAtFootInWorldFrame_x),'r');
plot(time, data(:,idx_loco_leftHindLeg_contactForceAtFootInWorldFrame_y),'g');
plot(time, data(:,idx_loco_leftHindLeg_contactForceAtFootInWorldFrame_z),'b');
plot(time, sqrt((data(:,idx_loco_leftHindLeg_contactForceAtFootInWorldFrame_x).^2+data(:,idx_loco_leftHindLeg_contactForceAtFootInWorldFrame_y).^2+data(:,idx_loco_leftHindLeg_contactForceAtFootInWorldFrame_z).^2)),'k');
legend('fx', 'fy', 'fz', 'norm(f)');
xlabel('t [s]');

subplot(4, 1, 4);
title('RH'); hold on; grid on;
plot(time, 6000*data(:,idx_loco_rightHindLeg_stancePhase),'m');
plot(time, data(:,idx_loco_rightHindLeg_contactForceAtFootInWorldFrame_x),'r');
plot(time, data(:,idx_loco_rightHindLeg_contactForceAtFootInWorldFrame_y),'g');
plot(time, data(:,idx_loco_rightHindLeg_contactForceAtFootInWorldFrame_z),'b');
plot(time, sqrt((data(:,idx_loco_rightHindLeg_contactForceAtFootInWorldFrame_x).^2+data(:,idx_loco_rightHindLeg_contactForceAtFootInWorldFrame_y).^2+data(:,idx_loco_rightHindLeg_contactForceAtFootInWorldFrame_z).^2)),'k');
legend('fx', 'fy', 'fz', 'norm(f)');
xlabel('t [s]');



%% 
named_figure('Roco Swing'), clf

subplot(3, 1, 1);
title('x'); hold on; grid on;
plot(time, data(:,idx_roco_sea_test_taskSpacePositionsLf_x),'b');
plot(time, data(:,idx_roco_sea_test_desTaskSpacePositionsLf_x),'r');
xlabel('t [s]');

subplot(3, 1, 2);
title('y'); hold on; grid on;
plot(time, data(:,idx_roco_sea_test_taskSpacePositionsLf_y),'b');
plot(time, data(:,idx_roco_sea_test_desTaskSpacePositionsLf_y),'r');
xlabel('t [s]');

subplot(3, 1, 3);
title('z'); hold on; grid on;
plot(time, data(:,idx_roco_sea_test_taskSpacePositionsLf_z),'b');
plot(time, data(:,idx_roco_sea_test_desTaskSpacePositionsLf_z),'r');
xlabel('t [s]');

%%
if (plotCurrents)
    named_figure('currents'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_sea_LF_HAA_state_current),'b')
    %plot(time, data(:,idx_sea_LF_HAA_commanded_current),'r')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_sea_LF_HFE_state_current),'b')
   % plot(time, data(:,idx_sea_LF_HFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_sea_LF_KFE_state_current),'b')
    %plot(time, data(:,idx_sea_LF_KFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_sea_RF_HAA_state_current),'b')
   % plot(time, data(:,idx_sea_RF_HAA_commanded_current),'r')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_sea_RF_HFE_state_current),'b')
   % plot(time, data(:,idx_sea_RF_HFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_sea_RF_KFE_state_current),'b')
   % plot(time, data(:,idx_sea_RF_KFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_sea_LH_HAA_state_current),'b')
    %plot(time, data(:,idx_sea_LH_HAA_commanded_current),'r')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_sea_LH_HFE_state_current),'b')
    %plot(time, data(:,idx_sea_LH_HFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_sea_LH_KFE_state_current),'b')
  %  plot(time, data(:,idx_sea_LH_KFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_sea_RH_HAA_state_current),'b')
  %  plot(time, data(:,idx_sea_RH_HAA_commanded_current),'r')
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_sea_RH_HFE_state_current),'b')
   % plot(time, data(:,idx_sea_RH_HFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_sea_RH_KFE_state_current),'b')
   % plot(time, data(:,idx_sea_RH_KFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

end

%% gear encoder
if (plotGearPositions)
    named_figure('measured gear positions'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_log_LF_HAA_th),'k.-')
    plot(time, data(:,idx_log_LF_HAA_th)+data(:,idx_log_LF_HAA_load).*(1/162.3),'b.-')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_log_LF_HFE_th),'k.-')
    plot(time, data(:,idx_log_LF_HFE_th)+data(:,idx_log_LF_HFE_load).*(1/162.3),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_log_LF_KFE_th),'k.-')
    plot(time, data(:,idx_log_LF_KFE_th)+data(:,idx_log_LF_KFE_load).*(1/162.3),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_log_RF_HAA_th),'k.-')
    plot(time, data(:,idx_log_RF_HAA_th)+data(:,idx_log_RF_HAA_load).*(1/162.3),'b.-')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,6)
    hold on

    plot(time, data(:,idx_log_RF_HFE_th),'k.-')
    plot(time, data(:,idx_log_RF_HFE_th)+data(:,idx_log_RF_HFE_load).*(1/162.3),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_log_RF_KFE_th),'k.-')
    plot(time, data(:,idx_log_RF_KFE_th)+data(:,idx_log_RF_KFE_load).*(1/162.3),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_log_LH_HAA_th),'k.-')
    plot(time, data(:,idx_log_LH_HAA_th)+data(:,idx_log_LH_HAA_load).*(1/162.3),'b.-')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_log_LH_HFE_th),'k.-')
    plot(time, data(:,idx_log_LH_HFE_th)+data(:,idx_log_LH_HFE_load).*(1/162.3),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_log_LH_KFE_th),'k.-')
    plot(time, data(:,idx_log_LH_KFE_th)+data(:,idx_log_LH_KFE_load).*(1/162.3),'b.-')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_log_RH_HAA_des_th),'r.-')
    plot(time, data(:,idx_log_RH_HAA_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_log_RH_HFE_des_th),'r.-')
    plot(time, data(:,idx_log_RH_HFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_log_RH_KFE_des_th),'r.-')
    plot(time, data(:,idx_log_RH_KFE_th),'b.-')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

end


%% Hip positions

%% Mode
if (plotLocoControlModes)
    named_figure('Position foot to hip in world frame'),clf
    grid on

    subplot(2,2,1)
    hold on
    positionFootToHipInWorldFrame_x = data(:,idx_loco_leftForeLeg_positionWorldToHipInWorldFrame_z) ...
                                    - data(:,idx_loco_leftForeLeg_positionWorldToFootInWorldFrame_z);
    plot(time, positionFootToHipInWorldFrame_x,'b')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('p [m]')
    
    subplot(2,2,2)
    hold on
    positionFootToHipInWorldFrame_x = data(:,idx_loco_rightForeLeg_positionWorldToHipInWorldFrame_z) ...
                                    - data(:,idx_loco_rightForeLeg_positionWorldToFootInWorldFrame_z);
    plot(time, positionFootToHipInWorldFrame_x,'b')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('p [m]')
    
    subplot(2,2,3)
    hold on
    positionFootToHipInWorldFrame_x = data(:,idx_loco_leftHindLeg_positionWorldToHipInWorldFrame_z) ...
                                    - data(:,idx_loco_leftHindLeg_positionWorldToFootInWorldFrame_z);
    plot(time, positionFootToHipInWorldFrame_x,'b')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('p [m]')
    
    subplot(2,2,4)
    hold on
    positionFootToHipInWorldFrame_x = data(:,idx_loco_rightHindLeg_positionWorldToHipInWorldFrame_z) ...
                                    - data(:,idx_loco_rightHindLeg_positionWorldToFootInWorldFrame_z);
    plot(time, positionFootToHipInWorldFrame_x,'b')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('p [m]')
    
end

