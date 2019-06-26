function [logElemsOut, newIdxNamesList] = getElemsByIdxName(logElemsIn, idxNamesList, varargin)
%GETELEMSBYNAME Gets the elements whose names are in namesList
%   Any strings in varargin will narrow the search to those starting with
%   any of the specified strings.
%   Time elements are included by default at indexes 1 and 2.

newIdxNamesList = idxNamesList;
if ~isempty(varargin) 
  noConditions = length(varargin);
  boolNamesList = false(length(idxNamesList),1);
  for k = 1:noConditions
    if strcmp(varargin{k},"idx_time")
      warning("Time elements are added by default (indexes 1 and 2)");
    else
      boolNamesList = boolNamesList|startsWith(idxNamesList,varargin{k});
    end
  end
  newIdxNamesList = idxNamesList(boolNamesList);
end

newIdxNamesList = vertcat("idx_time_s","idx_time_ns",newIdxNamesList);

for k = 1:length(newIdxNamesList)
  logElemsOut(k,1) = logElemsIn(evalin('base',newIdxNamesList(k)));
end

end

