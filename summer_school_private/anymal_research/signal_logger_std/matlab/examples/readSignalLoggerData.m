%% Read data collected using the Signal Logger
% Author:   Dario Bellicoso
% Date:     1st Jun 2017

clc, close all
clear all


%% Configuration
startNo = 5;      % number of first data file (filename will be generated based on this number
endNo = startNo;      % number of last data file

folder = '';       % name of folder where the data files are stored

tStart = 0;         % show only measurements of time range [tStart tEnd]
tEnd = 300;

%% Data process
processData;

%% Plot data
plotMainTorso;