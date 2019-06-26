function out = readLogFileLine(in, line)
try
    currentData = sscanf(line(4:end), '%f')';
catch
    currentData = [];
end
% Resize matrix if necessary
lengthDifference = length(currentData) - size(in, 2);
if lengthDifference > 0
    out = [in zeros(size(in, 1), lengthDifference)];
else
    out = in;
end
out(end+1, 1:length(currentData)) = currentData;
% out(end, length(currentData)+1:end) = NaN;
