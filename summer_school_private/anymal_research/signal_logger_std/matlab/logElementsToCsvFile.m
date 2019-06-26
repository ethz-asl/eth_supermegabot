function [csvTable] = logElementsToCsvFile(logElements, filename)
% Converts a vector of log elements into a MATLAB table (may take long!).
% The table can be turned into a column-wise CSV file with writetable()
  
  idx_time_s = evalin('base', 'idx_time_s');
  idx_time_ns = evalin('base', 'idx_time_ns');
  
  % Get number of elements (table columns)
  noElements = length(logElements);
  
  % Get number of samples (table rows)
  noSamples = length(logElements(idx_time_s).data);
  
  % Generate row names from time
  % DISABLED - only works if each sample has a different timestamp
  %rowNames = cellstr(num2str(double(logElements(idx_time_s).data)+double(logElements(idx_time_ns).data)*1e-9,6));
  
  % Generate column names from elements
  varTypes = cell(noElements,1);
  varNames = cell(noElements,1);
  strippedNames = cell(noElements,1);

  for i=1:noElements
    strippedNames(i) = cellstr((strrep(logElements(i).name{1}(6:end),'/','_')));
    if(~isvarname(strippedNames{i}))
      warning(['Variable name ',strippedNames{i},' is not valid!']);
    end
    varNames(i) = strippedNames(i);
    varTypes{i} = logElements(i).dataType;
  end
  
  % Initialize table
  csvTable = table('Size',[noSamples noElements],'VariableTypes',varTypes,'VariableNames',varNames);
    
  % Fill in elements
  for i=1:noElements
    for k=1:logElements(i).noData
      csvTable{noSamples-logElements(i).divider*(k-1),strippedNames(i)} = logElements(i).data(end-k+1);
    end
  end
  
  writetable(csvTable,filename);

end

