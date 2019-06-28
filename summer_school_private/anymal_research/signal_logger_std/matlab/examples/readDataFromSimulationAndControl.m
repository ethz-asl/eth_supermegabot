%% Read data collected withing SL
% Author:   Christian Gehring
% Date:     6th Feb 2012
% Function:

clc,  close all
%clear all


%% Configuration


startNo = 89;       % number of first data file (filename will be generated based on this number
endNo = startNo;      % number of last data file

folder = '';       % name of folder where the data files are stored

tStart = 0;         % show only measurements of time range [tStart tEnd]
tEnd = 130;


% Plotting (1=activated 0=deactivated)
plotMainbodyPose = 1;
plotMainbodyVel =0;
plotMainbodyOmega = 0;
plotJointPositionsMeas =0;
plotJointPositionsMeasAndDes = 1;
plotJointVelocitiesMeas =0;
plotJointVelocitiesMeasAndDes = 0;
plotJointTorques = 0;
plotJointPower =0;



plotModes = 0;
plotContactFlags =0;


plotPhases = 0;
plotAPS = 0;
plotForces = 0;





%% Read data from file

 
[time, data, vars] = getDataFromFile(startNo, endNo, folder);

% % generate index variables
for k=1:length(vars)
    completeName = char(vars(k).name);
    [startIndex,endIndex] = regexp(completeName ,'/');
    
    if ~isempty(endIndex)
        strippedName = completeName(endIndex(end)+1:end);
    else
        strippedName = completeName;
    end
    disp([num2str(k) ' ' strippedName]);
    eval([' global idx_' strippedName]);
    eval(['idx_' strippedName '=' num2str(k) ';']);
end

% get time range
[time, data] = getDataInTimeRange(time, data, tStart, tEnd);

% get time index of mode switches
[idx_LF_m, idx_RF_m, idx_LH_m, idx_RH_m] = getIdxOfModeSwitch(data);

% get scaled modes for visualization
[LF_m_scaled RF_m_scaled LH_m_scaled RH_m_scaled] = getScaledMode(data, [2 3], [0 1]);

%% Plotting


%% Mainbody pose
if (plotMainbodyPose)
    named_figure('main body position')
    subplot(3,1,1)
    plot(time, data(:,idx_qX));
    %plotVLines(time, [0.3 2 10], {'r','b','g'}, true);
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,1,2)
    plot(time, data(:,idx_qY))
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,1,3)
    plot(time, data(:,idx_qZ))
    grid on
    xlabel('time [s]')
    ylabel('z')
    named_figure('main body orientation')
    subplot(3,1,1)
    plot(time, data(:,idx_qAL))
    grid on
    xlabel('time [s]')
    ylabel('alpha')
    subplot(3,1,2)
    plot(time, data(:,idx_qBE))
    grid on
    xlabel('time [s]')
    ylabel('beta')
    subplot(3,1,3)
    plot(time, data(:,idx_qGA))
    grid on
    xlabel('time [s]')
    ylabel('gamma')

end


%% Mainbody velocities
if (plotMainbodyVel)
    named_figure('main body translational velocity world')
    subplot(3,1,1)
    plot(time, data(:,idx_dqX))
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,1,2)
    plot(time, data(:,idx_dqY))
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,1,3)
    plot(time, data(:,idx_dqZ))
    grid on
    xlabel('time [s]')
    ylabel('z')
    named_figure('main body rotational velocity')
    subplot(3,1,1)
    plot(time, data(:,idx_dqAL))
    grid on
    xlabel('time [s]')
    ylabel('alpha')
    subplot(3,1,2)
    plot(time, data(:,idx_dqBE))
    grid on
    xlabel('time [s]')
    ylabel('beta')
    subplot(3,1,3)
    plot(time, data(:,idx_dqGA))
    grid on
    xlabel('time [s]')
    ylabel('gamma')

end


%% Mainbody omega
if (plotMainbodyVel)
     named_figure('main body translational velocity base')
    subplot(3,1,1)
    plot(time, data(:,idx_mainBodyVelCSmb_0_0))
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,1,2)
    plot(time, data(:,idx_mainBodyVelCSmb_1_0))
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,1,3)
    plot(time, data(:,idx_mainBodyVelCSmb_2_0))
    grid on
    xlabel('time [s]')
    ylabel('z')
    
    
    named_figure('main body omega')
    subplot(3,1,1)
    plot(time, data(:,idx_omegaEst_x))
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,1,2)
    plot(time, data(:,idx_omegaEst_y))
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,1,3)
    plot(time, data(:,idx_omegaEst_z))
    grid on
    xlabel('time [s]')
    ylabel('z')
   

end


%% Measurred joint positions
if (plotJointPositionsMeas)
    
    named_figure('Joints LF'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_LF_HAA_th))

    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_LF_HFE_th))

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_LF_KFE_th))

    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    named_figure('Joints RF'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_RF_HAA_th))
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_RF_HFE_th))
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_RF_KFE_th))
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    named_figure('Joints LH'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_LH_HAA_th))
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_LH_HFE_th))
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_LH_KFE_th))
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')


    named_figure('Joints RH'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_RH_HAA_th))
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_RH_HFE_th))
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_RH_KFE_th))
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
    plot(time, data(:,idx_LF_HAA_des_th),'r')
    plot(time, data(:,idx_LF_HAA_th),'b')

    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_des_th),'r')
    plot(time, data(:,idx_LF_HFE_th),'b')

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_des_th),'r')
    plot(time, data(:,idx_LF_KFE_th),'b')

    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_des_th),'r')
    plot(time, data(:,idx_RF_HAA_th),'b')
  
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_des_th),'r')
    plot(time, data(:,idx_RF_HFE_th),'b')

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_des_th),'r')
    plot(time, data(:,idx_RF_KFE_th),'b')

    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_des_th),'r')
    plot(time, data(:,idx_LH_HAA_th),'b')

    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_des_th),'r')
    plot(time, data(:,idx_LH_HFE_th),'b')

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_des_th),'r')
    plot(time, data(:,idx_LH_KFE_th),'b')

    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_des_th),'r')
    plot(time, data(:,idx_RH_HAA_th),'b')

    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_des_th),'r')
    plot(time, data(:,idx_RH_HFE_th),'b')

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_des_th),'r')
    plot(time, data(:,idx_RH_KFE_th),'b')
 
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
    plot(time, data(:,idx_LF_HAA_thd),'b')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_thd),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_thd),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_thd),'b')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_thd),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_thd),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_thd),'b')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_thd),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_thd),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_thd),'b')
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_thd),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_thd),'b')
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
    plot(time, data(:,idx_LF_HAA_thd),'b')
    plot(time, data(:,idx_LF_HAA_des_thd),'r')

    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_thd),'b')
    plot(time, data(:,idx_LF_HFE_des_thd),'r')

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_thd),'b')
    plot(time, data(:,idx_LF_KFE_des_thd),'r')

    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_thd),'b')
    plot(time, data(:,idx_RF_HAA_des_thd),'r')

    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_thd),'b')
    plot(time, data(:,idx_RF_HFE_des_thd),'r')

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_thd),'b')
    plot(time, data(:,idx_RF_KFE_des_thd),'r')

    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_thd),'b')
    plot(time, data(:,idx_LH_HAA_des_thd),'r')

    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_thd),'b')
    plot(time, data(:,idx_LH_HFE_des_thd),'r')

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_thd),'b')
    plot(time, data(:,idx_LH_HFE_des_thd),'r')

    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_thd),'b')
    plot(time, data(:,idx_RH_HAA_des_thd),'r')
 
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_thd),'b')
    plot(time, data(:,idx_RH_HFE_des_thd),'r')

    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_thd),'b')
    plot(time, data(:,idx_RH_HFE_des_thd),'r')

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
    plot(time, data(:,idx_LF_HAA_load),'b')
    plot(time, data(:,idx_LF_HAA_uff), 'r')

    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_load),'b')
    plot(time, data(:,idx_LF_HFE_uff),'r')

    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_load),'b')
    plot(time, data(:,idx_LF_KFE_uff),'r')

    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_load),'b')
    plot(time, data(:,idx_RF_HAA_uff),'r')

    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_load),'b')
    plot(time, data(:,idx_RF_HFE_uff),'r')

    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_load),'b')
    plot(time, data(:,idx_RF_KFE_uff),'r')

    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_load),'b')
    plot(time, data(:,idx_LH_HAA_uff),'r')

    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_load),'b')
    plot(time, data(:,idx_LH_HFE_uff),'r')

    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_load),'b')
    plot(time, data(:,idx_LH_KFE_uff),'r')

    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_load),'b')
    plot(time, data(:,idx_RH_HAA_uff),'r')

    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_load),'b')
    plot(time, data(:,idx_RH_HFE_uff),'r')

    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_load),'b')
    plot(time, data(:,idx_RH_KFE_uff),'r')

    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

end
%% Mode
if (plotModes)
    named_figure('joint modes'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_LF_HAA_m),'b')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_m),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_m),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_m),'b')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_m),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_m),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_m),'b')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_m),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_m),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_m),'b')
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_m),'b')
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_m),'b')
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')
end


%% Forces

if (plotForces)
    named_figure('legsVForce'), clf
    grid on
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_frontLegsVForce_x)+data(:,idx_rearLegsVForce_x)+data(:,idx_allLegsVForce_x),'k')
    plot(time, data(:,idx_frontLegsVForce_x),'b')
    plot(time, data(:,idx_rearLegsVForce_x),'g')
    plot(time, data(:,idx_allLegsVForce_x),'r')
    plot(time, data(:,idx_netStanceForce_x),'m')

    legend('total','frontLegsVForce','rearLegsVForce','allLegsVForce','total distributed')
    title('LegsVForce')
    grid on
    xlabel('time [s]')
    ylabel('sagittal (x) [N]')
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_frontLegsVForce_y)+data(:,idx_rearLegsVForce_y)+data(:,idx_allLegsVForce_y),'k')
    plot(time, data(:,idx_frontLegsVForce_y),'b')
    plot(time, data(:,idx_rearLegsVForce_y),'g')
    plot(time, data(:,idx_allLegsVForce_y),'r')
    plot(time, data(:,idx_netStanceForce_y),'m')

    grid on
    xlabel('time [s]')
    ylabel('coronal (y) [N]')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_frontLegsVForce_z)+data(:,idx_rearLegsVForce_z)+data(:,idx_allLegsVForce_z),'k')
    plot(time, data(:,idx_frontLegsVForce_z),'b')
    plot(time, data(:,idx_rearLegsVForce_z),'g')
    plot(time, data(:,idx_allLegsVForce_z),'r')
    plot(time, data(:,idx_netStanceForce_z),'m')

    grid on
    xlabel('time [s]')
    ylabel('vertical (z) [N]')


    named_figure('netTorque'), clf
    grid on
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_netVTorque_x)+data(:,idx_netPDTorque_x),'k')
    plot(time, data(:,idx_netVTorque_x),'b')
    plot(time, data(:,idx_netPDTorque_x),'r')
    %plot(time, ones(length(time),1)*mean(data(:,idx_netPDTorque_x)),'r--')
    %plot(time, ones(length(time),1)*mean(data(:,idx_netVDTorque_x)),'c--')
    plot(time, data(:,idx_netStanceTorque_x),'m')
    legend('total net torque','virtual','PD','distributed torque')

    title('netTorque')
    grid on
    xlabel('time [s]')
    ylabel('roll (x) [Nm]')
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_netVTorque_y)+data(:,idx_netPDTorque_y),'k')
    plot(time, data(:,idx_netVTorque_y),'b')
    plot(time, data(:,idx_netPDTorque_y),'r')
    plot(time, data(:,idx_netStanceTorque_y),'m')

    grid on
    xlabel('time [s]')
    ylabel('pitch (y) [Nm]')

    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_netVTorque_z)+data(:,idx_netPDTorque_z),'k')
    plot(time, data(:,idx_netVTorque_z),'b')
    plot(time, data(:,idx_netPDTorque_z),'r')
    plot(time, data(:,idx_netStanceTorque_z),'m')

    grid on
    xlabel('time [s]')
    ylabel('yaw (z) [Nm]')


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
    plot(time, data(:,idx_LF_CONTACT_FLAG),'b')
    title('LF')
    grid on
    subplot(4,1,2)
    hold on
    plot(time, data(:,idx_RF_CONTACT_FLAG),'b')
    title('RF')
    grid on
    subplot(4,1,3)
    hold on
    plot(time, data(:,idx_LH_CONTACT_FLAG),'b')
    grid on
    title('LH')
    subplot(4,1,4)
    hold on
    plot(time, data(:,idx_RH_CONTACT_FLAG),'b')
    grid on
    title('RH')
end
    

%%
if (plotJointPower)
    
    LF_HAA_power = data(:,idx_LF_HAA_load).*data(:,idx_LF_HAA_thd);
    LF_HFE_power = data(:,idx_LF_HFE_load).*data(:,idx_LF_HFE_thd);
    LF_KFE_power = data(:,idx_LF_KFE_load).*data(:,idx_LF_KFE_thd);
    RF_HAA_power = data(:,idx_RF_HAA_load).*data(:,idx_RF_HAA_thd);
    RF_HFE_power = data(:,idx_RF_HFE_load).*data(:,idx_RF_HFE_thd);
    RF_KFE_power = data(:,idx_RF_KFE_load).*data(:,idx_RF_KFE_thd);
    LH_HAA_power = data(:,idx_LH_HAA_load).*data(:,idx_LH_HAA_thd);
    LH_HFE_power = data(:,idx_LH_HFE_load).*data(:,idx_LH_HFE_thd);
    LH_KFE_power = data(:,idx_LH_KFE_load).*data(:,idx_LH_KFE_thd);
    RH_HAA_power = data(:,idx_RH_HAA_load).*data(:,idx_RH_HAA_thd);
    RH_HFE_power = data(:,idx_RH_HFE_load).*data(:,idx_RH_HFE_thd);
    RH_KFE_power = data(:,idx_RH_KFE_load).*data(:,idx_RH_KFE_thd);
    
    power = [LF_HAA_power; LF_HFE_power; LF_KFE_power; RF_HAA_power; RF_HFE_power;  RF_KFE_power; LH_HAA_power; LH_HFE_power; LH_KFE_power; RH_HAA_power; RH_HFE_power; RH_KFE_power];
    meanPower = mean(power)
    
    named_figure('joint power'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, LF_HAA_power,'b')

    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,5)
    hold on
    plot(time, LF_HFE_power,'b')
 
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,9)
    hold on
    plot(time, LF_KFE_power,'b')
  
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_load).*data(:,idx_RF_HAA_thd),'b')

    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_load).*data(:,idx_RF_HFE_thd),'b')

    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_load).*data(:,idx_RF_KFE_thd),'b')

    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_load).*data(:,idx_LH_HAA_thd),'b')

    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_load).*data(:,idx_LH_HFE_thd),'b')

    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_load).*data(:,idx_LH_KFE_thd),'b')
 
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_load).*data(:,idx_RH_HAA_thd),'b')

    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_load).*data(:,idx_RH_HFE_thd),'b')

    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_load).*data(:,idx_RH_KFE_thd),'b')

    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')
end



