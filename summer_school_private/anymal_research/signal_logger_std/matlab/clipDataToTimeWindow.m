function [ leOut ] = clipDataToTimeWindow( leIn, startTime, timeWindow, logFreq)
% Returns a clipped logElement 

if nargin < 4
  logFreq = 400;
end

if startTime < leIn(1).time(1)
  error("startTime must be larger or equal than first logged time!");
end
if startTime+timeWindow > leIn(1).time(end)
  error("endTime must be smaller or equal than last logged time!");
end

% Set out equals in
leOut = leIn;

% Check buffer type
for i=1:length(leOut)
    startIdx = round((startTime-leIn(i).time(1))*logFreq)+1;
    stopIdx = startIdx+round(timeWindow*logFreq);
    
    leOut(i).timeStruct.seconds = leOut(i).timeStruct.seconds(startIdx:stopIdx);
    leOut(i).timeStruct.nanoseconds = leOut(i).timeStruct.nanoseconds(startIdx:stopIdx);
    leOut(i).systime = leOut(i).systime(startIdx:stopIdx);
    leOut(i).time = leOut(i).time(startIdx:stopIdx);
    leOut(i).data = leOut(i).data(startIdx:stopIdx);
    leOut(i).noData = length(leOut(i).data);
end

end

