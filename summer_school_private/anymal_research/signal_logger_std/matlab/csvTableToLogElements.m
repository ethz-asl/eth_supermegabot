function [ logElements ] = csvTableToLogElements(csvTable)
% Converts a row-wise CSV table (as provided by SiLo) into a vector of LogElement

% Get number of elements
  noElements = size(csvTable,1);
    
  % initialize log elements
  logElements = repmat(struct('name', '', 'noBytes', 0, 'noData', 0, ...
                              'divider', 0, 'isBufferLooping', 0, 'data', []) , noElements, 1 );
                 
  for i=1:noElements
    logElements(i).name = csvTable{i,1};
    logElements(i).noBytes = NaN;
    logElements(i).noData = size(csvTable,2)-1;
    logElements(i).divider = NaN;
    logElements(i).isBufferLooping = NaN;
    logElements(i).dataType = 'float';
    IsNan = ~isnan(csvTable{i,2:end});
    logElements(i).data = IsNan.*csvTable{i,2:end};
    logElements(i).timeStruct = struct('seconds', [], 'nanoseconds', []);
    logElements(i).systime = IsNan.*csvTable{1,2:end};
    logElements(i).time = IsNan.*csvTable{1,2:end};
  end

end

