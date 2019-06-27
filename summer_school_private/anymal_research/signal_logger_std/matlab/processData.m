% Get filename from directory
fNumber = 5;
fName = getFilenameFromNumber(fNumber, ['/home/', getenv('LOGNAME'), '/.ros']);
fprintf(['\nGot filename: ', fName, ' from number: ', num2str(fNumber)]);

% Read data
logElements = loadLogFile(fName);
fprintf(['\n\nLoaded data from binary file:', fName]);

% Generate Index Variables
verbose = false;
genIndexVariables(logElements, verbose);
fprintf('\n\nGenerated indices for the log elements!\n');

% Increase precision in data cursor and show index of data point
set(0,'defaultFigureCreateFcn',@(s,e)datacursorextra(s))
