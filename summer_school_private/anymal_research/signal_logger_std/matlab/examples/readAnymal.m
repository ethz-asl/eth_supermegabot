%% Read data collected withing SL
% Author:   Christian Gehring
% Date:     6th Feb 2012
% Function:

clc, close all
clear all


%% Configuration
%1994 1Hz
%1998 3Hz startNo = 2049;

% 2059 freegait square up with orientation
% failure: 123
% 126: 2Hz swing test
% 128: 3Hz swing test

startNo = 5;      % number of first data file (filename will be generated based on this number
endNo = startNo;      % number of last data file

folder = '';       % name of folder where the data files are stored

tStart = 0;         % show only measurements of time range [tStart tEnd]
tEnd = 300;


% Plotting (1=activated 0=deactivated)
plotMainbodyPoseMeas = 0;
plotMainbodyPoseMeasAndDes = 1;
plotMainbodyVel = 1;
plotMainbodyOmega = 0;
plotJointPositionsMeas = 0;
plotJointPositionsMeasAndDes = 1;
plotJointVelocitiesMeas = 0;
plotJointVelocitiesMeasAndDes = 1;
plotJointTorques = 1;
plotMotorVelocitiesMeas = 0;
plotMotorVelocitiesMeasAnDes = 0;
plotMotorPower = 0;
plotCurrents = 1;
plotJointPower = 0;
plotOptoforces = 0;
plotMainbodyPositionFromMocap = 0;

plotContactFlags = 0;
plotDeflections = 0;

plotTerrain = 0;
plotDesiredFootPositions = 1;

plotEstimatedContactForce = 1;
plotTaskSpacePositionError = 1;

% Loco
plotLoco = 1;
if (plotLoco)
    plotLocoVMCandCFD = 1;
    plotLocoCFD = 1;
    plotLocoLegState = 1;
    plotDesiredFootPositions = 1;
    plotTerrain = 1;
    plotLocoControlModes = 1;
    plotLocoIsGrounded = 1;
    plotLocoTorso = 1;
else
    
    plotLocoVMCandCFD = 0;
    plotLocoCFD = 0;
    plotLocoLegState = 0;
    plotLocoControlModes = 0;
    plotLocoIsGrounded = 0;
    plotLocoTorso = 0;
end

plotPhases = 0;
plotAPS = 0;


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
    
    % hack
    strippedName = strrep(completeName, '/', '_');
    
    disp([num2str(k) ' ' strippedName]);
    eval([' global idx' strippedName]);
    eval(['idx' strippedName '=' num2str(k) ';']);
end

% get time range
[time, data] = getDataInTimeRange(time, data, tStart, tEnd);

% get time index of mode switches
[idx_LF_m, idx_RF_m, idx_LH_m, idx_RH_m] = getIdxOfModeSwitch(data);

% get scaled modes for visualization
[LF_m_scaled RF_m_scaled LH_m_scaled RH_m_scaled] = getScaledMode(data, [2 3], [0 1]);

%% Plotting


