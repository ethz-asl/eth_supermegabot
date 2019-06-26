%% Read data collected withing SLc
% Author:   Christian Gehring
% Date:     6th Feb 2012
% Function:

clc
close all
clear all


%% Configuration

% 286 LH_HFE current_id (before CommutationCalibration)
% 287 LH_HFE torque step (before CommutationCalibration)
% 289 LH_HFE current_id (after CommutationCalibration) 

% 341 interrupt in other variables
% 349 LH_HAA corrupt

% 376 0x04 with zero_offset=0.36
% 377 0x04 with zero_offset=0.486
% 442 1kHz 0.05/0.01
startNo = 68;       % number of first data file (filename will be generated based on this number
endNo = startNo;      % number of last data file

folder = '';       % name of folder where the data files are stored

tStart = 0;         % show only measurements of time range [tStart tEnd]
tEnd = 130;


% Plotting (1=activated 0=deactivated)
plotJointPositionsMeasAndDes = 1;
plotJointVelocitiesMeasAndDes = 1;
plotTorques = 1;
plotCurrents = 1;
plotModes = 1;

ANYDRIVE_LF_HAA = 0;
ANYDRIVE_LF_HFE = 0;
ANYDRIVE_LF_KFE = 0;
ANYDRIVE_RF_HAA = 0;
ANYDRIVE_RF_HFE = 0;
ANYDRIVE_RF_KFE = 0;
ANYDRIVE_LH_HAA = 0;
ANYDRIVE_LH_HFE = 0;
ANYDRIVE_LH_KFE = 0;
ANYDRIVE_RH_HAA = 0;
ANYDRIVE_RH_HFE = 0;
ANYDRIVE_RH_KFE = 0;

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
    
    % hack
    strippedName = strrep(completeName, '/', '_');
    
    disp([num2str(k) ' ' strippedName]);
    eval([' global idx' strippedName]);
    eval(['idx' strippedName '=' num2str(k) ';']);
end

% get time range
[time, data] = getDataInTimeRange(time, data, tStart, tEnd);


%% Plotting

%% Measured and desired joint positions
if (plotJointPositionsMeasAndDes)
    named_figure('measured and desired joint positions'), clf
    grid on
    subplot(3,4,1)
    hold on
    if (ANYDRIVE_LF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_commanded_jointPosition),'r')
    title('LF')
    end
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,5)
    hold on
    if (ANYDRIVE_LF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_commanded_jointPosition),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,9)
    hold on
    if (ANYDRIVE_LF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_commanded_jointPosition),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,2)
    hold on
    if (ANYDRIVE_RF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_commanded_jointPosition),'r')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,6)
    hold on
    if (ANYDRIVE_RF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_commanded_jointPosition),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,10)
    hold on
    if (ANYDRIVE_RF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_commanded_jointPosition),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,3)
    hold on
    if (ANYDRIVE_LH_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_commanded_jointPosition),'r')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,7)
    hold on
    if (ANYDRIVE_LH_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_commanded_jointPosition),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,11)
    hold on
    if (ANYDRIVE_LH_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_commanded_jointPosition),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

    subplot(3,4,4)
    hold on
    if (ANYDRIVE_RH_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_commanded_jointPosition),'r')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad]')
    subplot(3,4,8)
    hold on
    if (ANYDRIVE_RH_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_commanded_jointPosition),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad]')
    subplot(3,4,12)
    hold on
    if (ANYDRIVE_RH_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_state_jointPosition),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_commanded_jointPosition),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad]')

end


%% Measured and desired joint velocities
if (plotJointVelocitiesMeasAndDes)
    named_figure('measured and desired joint velocities'), clf
    grid on
    subplot(3,4,1)
    hold on
    if (ANYDRIVE_LF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_commanded_jointVelocity),'r')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    if (ANYDRIVE_LF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_commanded_jointVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    if (ANYDRIVE_LF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_commanded_jointVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    if (ANYDRIVE_RF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_commanded_jointVelocity),'r')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    if (ANYDRIVE_RF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_commanded_jointVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    if (ANYDRIVE_RF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_commanded_jointVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    if (ANYDRIVE_LH_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_commanded_jointVelocity),'r')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,7)
    hold on
    if (ANYDRIVE_LH_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_commanded_jointVelocity),'r')
    
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    if (ANYDRIVE_LH_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_commanded_jointVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    if (ANYDRIVE_RH_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_commanded_jointVelocity),'r')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_commanded_actuatorVelocity),'k')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    if (ANYDRIVE_RH_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_commanded_jointVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    if (ANYDRIVE_RH_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_commanded_jointVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

end

%% Measured and desired joint velocities
if (plotJointVelocitiesMeasAndDes)
    named_figure('measured and desired joint velocities mixed'), clf
    grid on
    subplot(3,4,1)
    hold on
    if (ANYDRIVE_LF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_commanded_actuatorVelocity),'r')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,5)
    hold on
    if (ANYDRIVE_LF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_commanded_actuatorVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,9)
    hold on
    if (ANYDRIVE_LF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_commanded_actuatorVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,2)
    hold on
    if (ANYDRIVE_RF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_commanded_actuatorVelocity),'r')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,6)
    hold on
    if (ANYDRIVE_RF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_commanded_actuatorVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,10)
    hold on
    if (ANYDRIVE_RF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_commanded_actuatorVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,3)
    hold on
    if (ANYDRIVE_LH_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_commanded_actuatorVelocity),'r')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,7)
    hold on
    if (ANYDRIVE_LH_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_commanded_actuatorVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,11)
    hold on
    if (ANYDRIVE_LH_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_commanded_actuatorVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

    subplot(3,4,4)
    hold on
    if (ANYDRIVE_RF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_commanded_actuatorVelocity),'r')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [rad/s]')
    subplot(3,4,8)
    hold on
    if (ANYDRIVE_RF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_commanded_actuatorVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [rad/s]')
    subplot(3,4,12)
    hold on
    if (ANYDRIVE_RF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_state_jointVelocity),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_commanded_actuatorVelocity),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [rad/s]')

end
%% Torques

if (plotTorques)
    named_figure('joint torques'),clf
    grid on
    subplot(3,4,1)
    hold on
    if (ANYDRIVE_LF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_commanded_torque),'r')
    end
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,5)
    hold on
    if (ANYDRIVE_LF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_commanded_torque),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,9)
    hold on
    if (ANYDRIVE_LF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_commanded_torque),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,2)
    hold on
    if (ANYDRIVE_RF_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_commanded_torque),'r')
    end
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,6)
    hold on
    if (ANYDRIVE_RF_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_commanded_torque),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,10)
    hold on
    if (ANYDRIVE_RF_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_commanded_torque),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,3)
    hold on
    if (ANYDRIVE_LH_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_commanded_torque),'r')
    end
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,7)
    hold on
    if (ANYDRIVE_LH_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_commanded_torque),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,11)
    hold on
    if (ANYDRIVE_LH_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_commanded_torque),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

    subplot(3,4,4)
    hold on
    if (ANYDRIVE_RH_HAA)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_commanded_torque),'r')
    end
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [Nm]')
    subplot(3,4,8)
    hold on
    if (ANYDRIVE_RH_HFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_commanded_torque),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('HFE [Nm]')
    subplot(3,4,12)
    hold on
    if (ANYDRIVE_RH_KFE)
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_state_torque),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_commanded_torque),'r')
    end
    grid on
    xlabel('time [s]')
    ylabel('KFE [Nm]')

end

%% Currents

if (plotCurrents)
    named_figure('currents'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_commanded_current),'r')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_commanded_current),'r')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_commanded_current),'r')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_commanded_current),'r')
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_state_current),'b')
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_commanded_current),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

end


%% Modes

if (plotModes)
    named_figure('Modes'),clf
    grid on
    subplot(3,4,1)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HAA_commanded_mode),'r')
    title('LF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,5)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_commanded_mode),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,9)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_commanded_mode),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,2)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HAA_commanded_mode),'r')
    title('RF')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,6)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RF_HFE_commanded_mode),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,10)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RF_KFE_commanded_mode),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,3)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HAA_commanded_mode),'r')
    title('LH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,7)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_commanded_mode),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,11)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_LH_KFE_commanded_mode),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

    subplot(3,4,4)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_commanded_mode),'r')
    title('RH')
    grid on
    xlabel('time [s]')
    ylabel('HAA [A]')
    subplot(3,4,8)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RH_HFE_commanded_mode),'r')
    grid on
    xlabel('time [s]')
    ylabel('HFE [A]')
    subplot(3,4,12)
    hold on
    plot(time, data(:,idx_sea_ANYDRIVE_RH_KFE_commanded_mode),'r')
    grid on
    xlabel('time [s]')
    ylabel('KFE [A]')

end

%%
% figure
% plot(time, data(:,idx_sea_ANYDRIVE_LF_HFE_state_statusword))
% %plot(time, data(:,idx_sea_ANYDRIVE_LF_KFE_can_statusword))
% grid on

%%
% figure
% hold on
% %plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_jointPosition), 'r')
% plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_jointPosition), 'r')
% plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_jointVelocity), 'b')
% legend('jointPosition','jointVelocity')
% grid on

%%

% named_figure('debug'),clf
% hold on
% %plot(time, data(:,idx_sea_ANYDRIVE_LH_HFE_state_jointPosition), 'r')
% plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_can_debug_joint_position), 'r')
% plot(time, -data(:,idx_sea_ANYDRIVE_RH_HAA_can_debug_joint_velocity), 'b--')
% plot(time, data(:,idx_sea_ANYDRIVE_RH_HAA_can_debug_torque), 'k')
% legend('jointPosition','jointVelocity')
% grid on

%%
% encoder_resolution_ticks = int32(131072);
% threshold_multiturn = int32((131072 - 1000));
% AksIMHandle_ENC_ABS_JOINT_position = int32(data(2:end,idx_sea_ANYDRIVE_LH_KFE_can_debug_joint_position));
% AksIMHandle_ENC_ABS_MOTOR_position = int32(data(2:end,idx_sea_ANYDRIVE_LH_KFE_can_debug_joint_velocity));
% 
% enc_abs_dinitPosition_ticks = int32(data(2:end,idx_sea_ANYDRIVE_LH_KFE_can_debug_voltage));
% 
% enc_abs_position_ticks_ENC_ABS_MOTOR =  - AksIMHandle_ENC_ABS_JOINT_position + encoder_resolution_ticks;
% enc_abs_position_ticks_ENC_ABS_JOINT = AksIMHandle_ENC_ABS_MOTOR_position;
% 
% enc_abs_position_last_ticks_ENC_ABS_MOTOR = -int32(data(1:end-1,idx_sea_ANYDRIVE_LH_KFE_can_debug_joint_velocity))+ encoder_resolution_ticks;
% enc_abs_position_last_ticks_ENC_ABS_JOINT = int32(data(1:end-1,idx_sea_ANYDRIVE_LH_KFE_can_debug_joint_position));
% 
% delta_p_ENC_ABS_MOTOR = int32(enc_abs_position_last_ticks_ENC_ABS_MOTOR - enc_abs_position_ticks_ENC_ABS_MOTOR);
% delta_p_ENC_ABS_JOINT = int32(enc_abs_position_last_ticks_ENC_ABS_JOINT - enc_abs_position_ticks_ENC_ABS_JOINT);
% 
% enc_abs_turn_ENC_ABS_MOTOR = int32(zeros(length(delta_p_ENC_ABS_MOTOR), 1));
% enc_abs_turn_ENC_ABS_JOINT = int32(zeros(length(delta_p_ENC_ABS_MOTOR), 1));
% 
% for k=1:length(delta_p_ENC_ABS_MOTOR)
%    if ( delta_p_ENC_ABS_MOTOR(k) >= threshold_multiturn)
%        enc_abs_turn_ENC_ABS_MOTOR(k) = enc_abs_turn_ENC_ABS_MOTOR(k)  + 1;
%    end
%    if ( delta_p_ENC_ABS_MOTOR(k) <= threshold_multiturn)
%        enc_abs_turn_ENC_ABS_MOTOR(k) = enc_abs_turn_ENC_ABS_MOTOR(k)  - 1;
%    end
% end
% 
% for k=1:length(delta_p_ENC_ABS_JOINT)
%    if ( delta_p_ENC_ABS_JOINT(k) >= threshold_multiturn)
%        enc_abs_turn_ENC_ABS_JOINT(k) = enc_abs_turn_ENC_ABS_JOINT(k) + 1;
%    end
%    if ( delta_p_ENC_ABS_MOTOR(k) <= threshold_multiturn)
%        enc_abs_turn_ENC_ABS_JOINT(k) = enc_abs_turn_ENC_ABS_JOINT(k) - 1;
%    end
% end
% 
% enc_abs_position_multiturn_ticks_ENC_ABS_MOTOR = int32(enc_abs_position_ticks_ENC_ABS_MOTOR + enc_abs_turn_ENC_ABS_MOTOR * encoder_resolution_ticks);
% enc_abs_position_multiturn_ticks_ENC_ABS_JOINT = int32(enc_abs_position_ticks_ENC_ABS_JOINT + enc_abs_turn_ENC_ABS_JOINT * encoder_resolution_ticks);
% 
% deflection_actual_ticks = enc_abs_position_multiturn_ticks_ENC_ABS_MOTOR - enc_abs_position_multiturn_ticks_ENC_ABS_JOINT -  enc_abs_dinitPosition_ticks;
% for k=1:length(deflection_actual_ticks)
% 	if(deflection_actual_ticks(k) > threshold_multiturn)
%         deflection_actual_ticks(k)  = deflection_actual_ticks(k) - encoder_resolution_ticks;
%     end
% 	if(deflection_actual_ticks(k) < -threshold_multiturn)
%         deflection_actual_ticks(k)  =   deflection_actual_ticks(k) + encoder_resolution_ticks;
%     end
% end
%%
%named_figure('mat code'),clf
%hold on
%%plot(time(2:end), enc_abs_position_multiturn_ticks_ENC_ABS_JOINT,'r')
%%plot(time(2:end), enc_abs_position_multiturn_ticks_ENC_ABS_MOTOR,'b')
%plot(time(2:end), deflection_actual_ticks, 'k')
%grid on


figure()
hold on
plot(time,data(:,idx_sea_ANYDRIVE_RF_KFE_can_debug_joint_velocity)/2^13/10*2*pi-mean(data(:,idx_sea_ANYDRIVE_RF_KFE_can_debug_joint_velocity)/2^13/10*2*pi), 'b')
plot(time,data(:,idx_sea_ANYDRIVE_RF_KFE_can_debug_joint_position)*50/2^17*2*pi-mean(data(:,idx_sea_ANYDRIVE_RF_KFE_can_debug_joint_position)*50/2^17*2*pi), 'r')
plot(time,data(:,idx_sea_ANYDRIVE_RF_KFE_can_debug_voltage)*50/2^17*2*pi-mean(data(:,idx_sea_ANYDRIVE_RF_KFE_can_debug_voltage)*50/2^17*2*pi), 'k')
grid on
%%plot(time(2:end), enc_abs_position_multiturn_ticks_ENC_ABS_JOINT,'r')
%%
figure()
hold on
plot(time,data(:,idx_sea_ANYDRIVE_RF_KFE_can_debug_torque)/90)
plot([time(1) time(end)], [333 333],'r')
ylabel('period [us]')
grid on
%%
figure()
hold on
plot(time(1:end),data(1:end,idx_sea_ANYDRIVE_RF_KFE_can_debug_torque),'b.-')
plot(time,data(:,idx_sea_ANYDRIVE_RF_KFE_commanded_torque),'r.-')
grid on


%%
figure()
plot(time,data(:,idx_sea_ANYDRIVE_LF_HFE_can_debug_torque),'b.-')
%plot(diff(data(:,idx_sea_ANYDRIVE_LF_KFE_can_debug_torque)))
%ylim([-4 4])
grid on


%%
figure()
plot(time,data(:,idx_sea_ANYDRIVE_RF_KFE_can_debug_joint_velocity)/1000)

%%
%%
figure()
hold on
plot(time,data(:,idx_sea_ANYDRIVE_LF_HFE_can_debug_torque),'b.-')
plot(time,data(:,idx_sea_ANYDRIVE_LF_HFE_state_jointPosition),'r.-')
plot(time,data(:,idx_sea_ANYDRIVE_LF_HFE_state_jointPosition)+(data(:,idx_sea_ANYDRIVE_LF_HFE_can_debug_torque)+1)*2*pi,'k.-')
ylim([-3*pi 3*pi])
%plot(diff(data(:,idx_sea_ANYDRIVE_LF_KFE_can_debug_torque)))
%ylim([-4 4])
grid on


%%
figure()
title('RF_KFE joint position')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_KFE_state_jointPosition),'b.-')
plot(time,data(:,idx_sea_RF_KFE_commanded_jointPosition),'r.-')
grid on


figure()
title('RF_KFE joint position debug')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_KFE_can_debug_joint_position),'b.-')
grid on


figure()
title('RF_KFE joint velocity')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_KFE_state_jointVelocity),'b.-')
plot(time,data(:,idx_sea_RF_KFE_commanded_jointVelocity),'r.-')
grid on

figure()
title('RF_KFE torque')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_KFE_state_torque),'b.-')
plot(time,data(:,idx_sea_RF_KFE_commanded_torque),'r.-')
grid on

%%
figure()
title('RF_KFE actuator velocity')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_KFE_state_actuatorVelocity),'b.-')
plot(time,data(:,idx_sea_RF_KFE_commanded_actuatorVelocity),'r.-')
grid on


%%
figure()
title('RH_KFE actuator velocity')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_KFE_state_actuatorVelocity),'b.-')
plot(time,data(:,idx_sea_RH_KFE_commanded_actuatorVelocity),'r.-')
grid on

%%
figure()
title('RF_HFE joint position')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_HFE_state_jointPosition),'b.-')
plot(time,data(:,idx_sea_RF_HFE_commanded_jointPosition),'r.-')
grid on


figure()
title('RF_HFE joint position debug')
hold on
%plot(time(1:end),data(1:end,idx_sea_RF_HFE_can_debug_joint_position),'b.-')
grid on


figure()
title('RF_HFE joint velocity')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_HFE_state_jointVelocity),'b.-')
plot(time,data(:,idx_sea_RF_HFE_commanded_jointVelocity),'r.-')
grid on

figure()
title('RF_HFE torque')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_HFE_state_torque),'b.-')
plot(time,data(:,idx_sea_RF_HFE_commanded_torque),'r.-')
grid on


%%
figure()
title('RF_KFE joint position')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_KFE_state_jointPosition),'k.-')
plot(time,data(:,idx_sea_RH_KFE_commanded_jointPosition),'c.-')
grid on


figure()
title('RH_KFE joint position debug')
hold on
%plot(time(1:end),data(1:end,idx_sea_RH_KFE_can_debug_joint_position),'b.-')
grid on


figure()
title('RH_KFE joint velocity')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_KFE_state_jointVelocity),'b.-')
plot(time,data(:,idx_sea_RH_KFE_commanded_jointVelocity),'r.-')
grid on

figure()
title('RH_KFE torque')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_KFE_state_torque),'b.-')
plot(time,data(:,idx_sea_RH_KFE_commanded_torque),'r.-')
grid on


%%
figure()
title('RF_HFE joint position')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_HFE_state_jointPosition),'b.-')
plot(time,data(:,idx_sea_RH_HFE_commanded_jointPosition),'r.-')
grid on


figure()
title('RH_HFE joint position debug')
hold on
%plot(time(1:end),data(1:end,idx_sea_RH_HFE_can_debug_joint_position),'b.-')
grid on


figure()
title('RH_HFE joint velocity')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_HFE_state_jointVelocity),'b.-')
plot(time,data(:,idx_sea_RH_HFE_commanded_jointVelocity),'r.-')
grid on

figure()
title('RH_HFE torque')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_HFE_state_torque),'b.-')
plot(time,data(:,idx_sea_RH_HFE_commanded_torque),'r.-')
grid on

%%
figure()
title('RF_HAA joint position')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_HAA_state_jointPosition),'b.-')
plot(time,data(:,idx_sea_RH_HAA_commanded_jointPosition),'r.-')
grid on


figure()
title('RH_HAA joint position debug')
hold on
%plot(time(1:end),data(1:end,idx_sea_RH_HAA_can_debug_joint_position),'b.-')
grid on


figure()
title('RH_HAA joint velocity')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_HAA_state_jointVelocity),'b.-')
plot(time,data(:,idx_sea_RH_HAA_commanded_jointVelocity),'r.-')
grid on

figure()
title('RH_HAA torque')
hold on
plot(time(1:end),data(1:end,idx_sea_RH_HAA_state_torque),'b.-')
plot(time,data(:,idx_sea_RH_HAA_commanded_torque),'r.-')
grid on


%%
figure()
title('RF_HAA joint position')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_HAA_state_jointPosition),'b.-')
plot(time,data(:,idx_sea_RF_HAA_commanded_jointPosition),'r.-')
grid on

figure()
title('RF_HAA joint velocity')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_HAA_state_jointVelocity),'b.-')
plot(time,data(:,idx_sea_RF_HAA_commanded_jointVelocity),'r.-')
grid on

figure()
title('RF_HAA torque')
hold on
plot(time(1:end),data(1:end,idx_sea_RF_HAA_state_torque),'b.-')
plot(time,data(:,idx_sea_RF_HAA_commanded_torque),'r.-')
grid on

