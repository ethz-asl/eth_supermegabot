%% Read data collected withing SL for HyQ
% Author:   Christian Gehring
% Date:     6th Feb 2012
% Function:

clc,  close all
%clear all


%% Configuration


startNo = 30;       % number of first data file (filename will be generated based on this number
endNo = startNo;      % number of last data file

folder = '';       % name of folder where the data files are stored

tStart = 0;         % show only measurements of time range [tStart tEnd]
tEnd = 130;


% Plotting (1=activated 0=deactivated)
plotMainbodyPose = 1;
plotMainbodyVel = 1;
plotMainbodyAcc = 1;
plotMainbodyOmega = 1;
plotJointPositionsMeas = 0;
plotJointPositionsMeasAndDes = 1;
plotJointVelocitiesMeas = 1;
plotJointVelocitiesMeasAndDes = 1;
plotJointTorques = 1;
plotMotorVelocitiesMeas = 0;
plotMotorPower = 0;
plotJointPower =1;


plotModes = 0;
plotContactFlags = 0;
plotDeflections = 0;


plotPhases = 0;
plotAPS = 0;
plotForces = 0;

plot3DPose = 0;
plotFootholdEstimates = 0;

% plot a vertical line when the control mode is switched
plotModeSwitches = 0;



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
    plot(time, data(:,idx_eulerZyxWorldToBase_z))
    grid on
    xlabel('time [s]')
    ylabel('yaw [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_eulerZyxWorldToBase_y))
    grid on
    xlabel('time [s]')
    ylabel('pitch [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_eulerZyxWorldToBase_x))
    grid on
    xlabel('time [s]')
    ylabel('roll [rad]')

end


%% Mainbody velocities
if (plotMainbodyVel)
    named_figure('main body linear velocity')
    subplot(3,1,1)
    plot(time, data(:,idx_dqX))
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
    plot(time, data(:,idx_dqY))
    grid on
    xlabel('time [s]')
    ylabel('y [m/s]')
    subplot(3,1,3)
    plot(time, data(:,idx_dqZ))
    grid on
    xlabel('time [s]')
    ylabel('z [m/s]')
    named_figure('main body angular velocity expressed in base frame')
    subplot(3,1,1)
    plot(time, data(:,idx_angVelBaseInBaseFrame_x))
    grid on
    xlabel('time [s]')
    ylabel('x [rad/s]')
    subplot(3,1,2)
    plot(time, data(:,idx_angVelBaseInBaseFrame_y))
    grid on
    xlabel('time [s]')
    ylabel('y [rad/s]')
    subplot(3,1,3)
    plot(time, data(:,idx_angVelBaseInBaseFrame_z))
    grid on
    xlabel('time [s]')
    ylabel('z [rad/s]')
end


%% Measurred joint positions
if (plotJointPositionsMeas)
    
    named_figure('Joints LF'), clf
    grid on
    subplot(3,1,1)
    plot(time, data(:,idx_LF_HAA_th))
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_LF_HFE_th))
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_LF_KFE_th))
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
    end
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
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_des_th),'r')
    plot(time, data(:,idx_LF_HFE_th),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_des_th),'r')
    plot(time, data(:,idx_LF_KFE_th),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_des_th),'r')
    plot(time, data(:,idx_RF_HAA_th),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_des_th),'r')
    plot(time, data(:,idx_RF_HFE_th),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_des_th),'r')
    plot(time, data(:,idx_RF_KFE_th),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_des_th),'r')
    plot(time, data(:,idx_LH_HAA_th),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_des_th),'r')
    plot(time, data(:,idx_LH_HFE_th),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_des_th),'r')
    plot(time, data(:,idx_LH_KFE_th),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_des_th),'r')
    plot(time, data(:,idx_RH_HAA_th),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_des_th),'r')
    plot(time, data(:,idx_RH_HFE_th),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_des_th),'r')
    plot(time, data(:,idx_RH_KFE_th),'b')
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
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_thd),'b')
    plot(time, data(:,idx_LF_HFE_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_thd),'b')
    plot(time, data(:,idx_LF_KFE_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_thd),'b')
    plot(time, data(:,idx_RF_HAA_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_thd),'b')
    plot(time, data(:,idx_RF_HFE_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_thd),'b')
    plot(time, data(:,idx_RF_KFE_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_thd),'b')
    plot(time, data(:,idx_LH_HAA_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_thd),'b')
    plot(time, data(:,idx_LH_HFE_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_thd),'b')
    plot(time, data(:,idx_LH_HFE_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_thd),'b')
    plot(time, data(:,idx_RH_HAA_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_thd),'b')
    plot(time, data(:,idx_RH_HFE_des_thd),'r')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_thd),'b')
    plot(time, data(:,idx_RH_HFE_des_thd),'r')
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
    plot(time, data(:,idx_LF_HAA_load),'b')
    plot(time, data(:,idx_LF_HAA_uff), 'r')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_load),'b')
    plot(time, data(:,idx_LF_HFE_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_load),'b')
    plot(time, data(:,idx_LF_KFE_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_load),'b')
    plot(time, data(:,idx_RF_HAA_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_load),'b')
    plot(time, data(:,idx_RF_HFE_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_load),'b')
    plot(time, data(:,idx_RF_KFE_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_load),'b')
    plot(time, data(:,idx_LH_HAA_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_load),'b')
    plot(time, data(:,idx_LH_HFE_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_load),'b')
    plot(time, data(:,idx_LH_KFE_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_load),'b')
    plot(time, data(:,idx_RH_HAA_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_load),'b')
    plot(time, data(:,idx_RH_HFE_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_load),'b')
    plot(time, data(:,idx_RH_KFE_uff),'r')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
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


%% Measured motor velocities
if (plotMotorVelocitiesMeas)
    named_figure('measured motor velocities'), clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_LF_HAA_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]/s')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_MOTOR),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

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
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
        vline(time(idx_RF_m),'m')
        vline(time(idx_LH_m),'b')
        vline(time(idx_RH_m),'c')
    end
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
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
        vline(time(idx_RF_m),'m')
        vline(time(idx_LH_m),'b')
        vline(time(idx_RH_m),'c')
    end
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
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
        vline(time(idx_RF_m),'m')
        vline(time(idx_LH_m),'b')
        vline(time(idx_RH_m),'c')
    end
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
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
        vline(time(idx_RF_m),'m')
        vline(time(idx_LH_m),'b')
        vline(time(idx_RH_m),'c')
    end
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
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
        vline(time(idx_RF_m),'m')
        vline(time(idx_LH_m),'b')
        vline(time(idx_RH_m),'c')
    end
    grid on
    xlabel('time [s]')
    ylabel('pitch (y) [Nm]')

    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_netVTorque_z)+data(:,idx_netPDTorque_z),'k')
    plot(time, data(:,idx_netVTorque_z),'b')
    plot(time, data(:,idx_netPDTorque_z),'r')
    plot(time, data(:,idx_netStanceTorque_z),'m')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'r')
        vline(time(idx_RF_m),'m')
        vline(time(idx_LH_m),'b')
        vline(time(idx_RH_m),'c')
    end
    grid on
    xlabel('time [s]')
    ylabel('yaw (z) [Nm]')


end

%%
if (plotDeflections)
    tol = 0.02;
    tolHigh = 0.07;
    
    named_figure('Deflections'), clf
    grid on
    subplot(1,4,1)
    hold on
    plot(time, data(:,idx_LF_KFE_DEFL),'b')
    hline(tol,'r')
    hline(tolHigh,'r')
    plot(time, data(:,idx_LF_CONTACT_FLAG)*tolHigh,'c-')
    grid on
    title('LF')
    xlabel('time [s]')
    ylabel('deflection [rad]')
    
    subplot(1,4,2)
    hold on
    plot(time, data(:,idx_RF_KFE_DEFL),'b')
    hline(tol,'r')
    hline(tolHigh,'r')
    plot(time, data(:,idx_RF_CONTACT_FLAG)*tolHigh,'c-')
    grid on
    title('RF')
    xlabel('time [s]')
    ylabel('deflection [rad]')
    
    subplot(1,4,3)
    hold on
    plot(time, -data(:,idx_LH_KFE_DEFL),'b')
    hline(tol,'r')
    hline(tolHigh,'r')
    plot(time, data(:,idx_LH_CONTACT_FLAG)*tolHigh,'c-')
    grid on
    title('LH')
    xlabel('time [s]')
    ylabel('deflection [rad]')
    
    subplot(1,4,4)
    hold on
    plot(time, -data(:,idx_RH_KFE_DEFL),'b')
    grid on
    hline(tol,'r')
    hline(tolHigh,'r')
    plot(time, data(:,idx_RH_CONTACT_FLAG)*tolHigh,'c-')
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
    
%% motor power
if (plotMotorPower)
    named_figure('motor power'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_LF_HAA_load).*data(:,idx_LF_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_LF_HFE_load).*data(:,idx_LF_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_LF_KFE_load).*data(:,idx_LF_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_RF_HAA_load).*data(:,idx_RF_HAA_MOTOR_VEL),'b')
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
    plot(time, data(:,idx_RF_KFE_load).*data(:,idx_RF_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_load).*data(:,idx_LH_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_load).*data(:,idx_LH_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_load).*data(:,idx_LH_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_load).*data(:,idx_RH_HAA_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_load).*data(:,idx_RH_HFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [W]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_load).*data(:,idx_RH_KFE_MOTOR_VEL),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [W]')
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
    plot(time, data(:,idx_RF_HAA_load).*data(:,idx_RF_HAA_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [W]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_RF_HFE_load).*data(:,idx_RF_HFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_RF_KFE_load).*data(:,idx_RF_KFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RF_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_LH_HAA_load).*data(:,idx_LH_HAA_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_LH_HFE_load).*data(:,idx_LH_HFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_LH_KFE_load).*data(:,idx_LH_KFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_LH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_RH_HAA_load).*data(:,idx_RH_HAA_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_RH_HFE_load).*data(:,idx_RH_HFE_thd),'b')
    if (plotModeSwitches)
        vline(time(idx_RH_m),'k')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_RH_KFE_load).*data(:,idx_RH_KFE_thd),'b')
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


%% Mainbody pose
if (plotMainbodyPose)
    named_figure('main body position (sim)'),clf
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_qX));
    plot(time, data(:, idx_BODY_X), 'r');
    %plotVLines(time, [0.3 2 10], {'r','b','g'}, true);
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_qY))
    plot(time, data(:, idx_BODY_Y), 'r');
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_qZ))
    plot(time, data(:, idx_BODY_Z), 'r');
    grid on
    xlabel('time [s]')
    ylabel('z')
    %%
    named_figure('main body orientation Quat'), clf
    subplot(4,1,1)
    hold on
    plot(time, data(:,idx_quatWorldToBase_w))
    plot(time, data(:,idx_BODY_Q_0), 'r')
    grid on
    xlabel('time [s]')
    ylabel('w]')
    subplot(4,1,2)
    hold on

    plot(time, data(:,idx_quatWorldToBase_x))
    plot(time, data(:,idx_BODY_Q_1), 'r')
    grid on
    xlabel('time [s]')
    ylabel('x')
    subplot(4,1,3)
        hold on
    plot(time, data(:,idx_quatWorldToBase_y))
    plot(time, data(:,idx_BODY_Q_2), 'r')
    grid on
    xlabel('time [s]')
    ylabel('y')
    subplot(4,1,4)
        hold on
    plot(time, data(:,idx_quatWorldToBase_z))
    plot(time, data(:,idx_BODY_Q_3), 'r')
    grid on
    xlabel('time [s]')
    ylabel('z')
      
    
    %%
    named_figure('main body orientation EulerZyx')
    subplot(3,1,1)
    plot(time, data(:,idx_eulerZyxWorldToBase_z))
    plot(time, data(:,idx_BODY_Q_3), 'r')
    grid on
    xlabel('time [s]')
    ylabel('yaw [rad]')
    subplot(3,1,2)
    plot(time, data(:,idx_eulerZyxWorldToBase_y))
    grid on
    xlabel('time [s]')
    ylabel('pitch [rad]')
    subplot(3,1,3)
    plot(time, data(:,idx_eulerZyxWorldToBase_x))
    grid on
    xlabel('time [s]')
    ylabel('roll [rad]')

end

%% Mainbody velocities
if (plotMainbodyVel)
    named_figure('main body linear velocity sim'), clf
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_dqX))
    plot(time, data(:,idx_linVelBaseCSw_x), 'r')
    grid on
    xlabel('time [s]')
    ylabel('x [m/s]')
    title('linear velocity of torso expressed in world frame')
    legend('estimated', 'ground truth')
    
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_dqY))
    plot(time, data(:,idx_linVelBaseCSw_y), 'r')
    grid on
    xlabel('time [s]')
    ylabel('y [m/s]')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_dqZ))
    plot(time, data(:,idx_linVelBaseCSw_z), 'r')
    grid on
    xlabel('time [s]')
    ylabel('z [m/s]')
    
    named_figure('main body angular velocity expressed in base frame '), clf
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_angVelBaseInBaseFrame_x))
    plot(time, data(:, idx_angVelBaseCSb_x), 'r')
    grid on
    xlabel('time [s]')
    ylabel('x [rad/s]')
    legend('estimated', 'ground truth')
    
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_angVelBaseInBaseFrame_y))
    plot(time, data(:, idx_angVelBaseCSb_y), 'r')
    grid on
    xlabel('time [s]')
    ylabel('y [rad/s]')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_angVelBaseInBaseFrame_z))
    plot(time, data(:, idx_angVelBaseCSb_z), 'r')
    hold on
    grid on
    xlabel('time [s]')
    ylabel('z [rad/s]')
end


%% Mainbody accelerations
if (plotMainbodyAcc)
    named_figure('main body linear acceleration sim'), clf
    subplot(3,1,1)
    hold on
    plot(time, data(:,idx_linAccBaseCSw_x), 'r')
    grid on
    xlabel('time [s]')
    ylabel('x [m/s^2]')
    title('linear acceleration of torso expressed in world frame')
    
    subplot(3,1,2)
    hold on
    plot(time, data(:,idx_linAccBaseCSw_y), 'r')
    grid on
    xlabel('time [s]')
    ylabel('y [m/s^2]')
    subplot(3,1,3)
    hold on
    plot(time, data(:,idx_linAccBaseCSw_z), 'r')
    grid on
    xlabel('time [s]')
    ylabel('z [m/s^2]')
    
end

