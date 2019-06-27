% This function imports the data from the confusion logger.
% * bagName: ConFusion data bag name
function [conFusionData] = importConFusionData(bagName)
tic
file = fopen(bagName);

batches = cell(0);
staticParameters = struct();
priorResiduals = [];
updateResidualBatches = cell(0);
processResidualBatches = cell(0);
userData = [];

% Process header
line = fgetl(file);
if strcmp(line, 'State names')
    usingStateNames = true;
    
    stateLengths = [];
    stateNames = cell(0);
    
    while true
        line = fgetl(file);
        if strcmp(line, 'Start log')
            break;
        end
        
        [currentStateLength, currentStateName] = strtok(line, ' ');
        currentStateName(isspace(currentStateName)) = [];
        
%         eval(['batches.' currentStateName ' = [];']);
        
        stateLengths(end+1, 1) = str2num(currentStateLength);
        stateNames{end+1} = currentStateName;
    end
elseif strcmp(line, 'Start log')
    usingStateNames = false;
    % empty
else
    error('Invalid first line');
end


line = fgetl(file);
while ischar(line)
    type = line(1:2);
    switch type
        case 'BS'
            % Batches need cells, as these can have different numbers of
            % states
            numBatch = sscanf(line(4:end), '%i');
            
            if (usingStateNames)
                % Initialize
                batches{end+1} = struct();
                batches{end}.time = [];
                for iState = 1:length(stateNames)
                     eval(['batches{end}.' stateNames{iState} ' = [];']);
                end

                for i = 1:numBatch
                    line = fgetl(file);
                    currentData = sscanf(line(4:end), '%f')';
                    % batches{end} = readLogFileLine(batches{end}, line);

                    batches{end}.time(end+1,1) = currentData(1);
                    endColumn = 1;
                    for iState = 1:length(stateNames)
                        startColumn = endColumn + 1;
                        endColumn = endColumn + stateLengths(iState);
                        eval(['batches{end}.' stateNames{iState} '(end+1, :) = currentData(startColumn:endColumn);']);
                    end
                end
            else
                batches{end+1} = [];
                for i = 1:numBatch
                    line = fgetl(file);
                    batches{end} = readLogFileLine(batches{end}, line);
                end
            end
            line = fgetl(file);
            type = line(1:2);
            code = line(4:5);
            if ~strcmp(type, 'BS') || ~strcmp(code, '-1')
                error('Batch ended in wrong order');
            end
        case 'SP'
            % Initialize
            numStaticParameters = sscanf(line(4:end), '%i');
            for i = 1:numStaticParameters
                line = fgetl(file);
                parameterName = strtok(line);
                parameterExists = eval(['isfield(staticParameters, ''' parameterName ''');']);
                if ~parameterExists
                    eval(['staticParameters.' parameterName '= [];']);
                end
                %line(length(parameterName)+2:end)
                eval(['staticParameters.' parameterName '(end+1,:) = sscanf(line(length(parameterName)+2:end), ''%f'');']);
            end
            if ~strcmp(type, 'SP') || ~strcmp(code, '-1')
                error('Static parameters ended in wrong order');
            end
        case 'PI'
            % Prior residuals grow over time
            priorResiduals = readLogFileLine(priorResiduals, line);
        case 'BU'
            numBatch = sscanf(line(4:end), '%i');
            updateResidualBatches{end+1} = [];
            for i = 1:numBatch
                line = fgetl(file);
                updateResidualBatches{end} = readLogFileLine(updateResidualBatches{end}, line);
            end
            line = fgetl(file);
            type = line(1:2);
            code = line(4:5);
            if ~strcmp(type, 'BU') || ~strcmp(code, '-1')
                error('Batch ended in wrong order');
            end
        case 'BP'
            numBatch = sscanf(line(4:end), '%i');
            processResidualBatches{end+1} = [];
            for i = 1:numBatch
                line = fgetl(file);
                processResidualBatches{end} = readLogFileLine(processResidualBatches{end}, line);
            end
            line = fgetl(file);
            type = line(1:2);
            code = line(4:5);
            if ~strcmp(type, 'BP') || ~strcmp(code, '-1')
                error('Batch ended in wrong order');
            end
        case 'UU'
            userData = readLogFileLine(userData, line);
        otherwise
            error('Unrecognized line format');
    end           
            
    line = fgetl(file);
end
toc

conFusionData.stateNames = stateNames;
conFusionData.stateLengths = stateLengths;
conFusionData.batches = batches;
conFusionData.staticParameters = staticParameters;
conFusionData.priorResiduals = priorResiduals;
conFusionData.updateResidualBatches = updateResidualBatches;
conFusionData.processResidualBatches = processResidualBatches;
conFusionData.userData = userData;

end
