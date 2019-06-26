% This function imports the data from the confusion logger and plots a
% simplified version of it, just showing the data of the newest state
% in the batch of each iteration.
% * bag_name: ConFusion data bag name (string)
% * plot_state: Plotting states (boolean)
% * plot_static_parameters: Plotting static parameters (boolean)
% * plot_residuals: Plotting residuals (boolean)
% * plot_user_data: Plotting user data (boolean) Only done in a separate
% figure currently
% * separate_figures: Separating the plots of the three options above into multiple figures (boolean)
function [confusion_data] = plotConFusionData(bag_name, plot_state, plot_static_parameters, plot_residuals, plot_user_data, separate_figures)
% Import data
confusion_data = importConFusionData(bag_name);
disp('Finished importing data.');

%% Plotting variables
num_state_params = size(confusion_data.stateNames, 2);
num_state_params_ext = sum(confusion_data.stateLengths);
num_static_params = length(fieldnames(confusion_data.staticParameters));
num_process_sensors = max(confusion_data.processResidualBatches{end}(:,2)) + 1;
num_update_sensors = max(confusion_data.updateResidualBatches{end}(:,2)) + 1;
static_param_names = fieldnames(confusion_data.staticParameters);
state_lengths = confusion_data.stateLengths;
state_names = confusion_data.stateNames;
num_plots = plot_state * num_state_params +  plot_static_parameters * num_static_params ...
    + plot_residuals * 3;
rows = ceil(sqrt(num_plots));
columns = ceil(num_plots / rows);
axes = zeros(num_plots);

% Plotting general
tot_plot_index = 1;
page_tot_plot_index = 1;
set(0,'defaultAxesColorOrder', [0 0 1; 1 0 0; 0 1 0; 0 1 1], ...
      'defaultAxesLineStyleOrder','-|--|:')
if not(separate_figures) 
    figure('name','ConFusion Analysis');
end

% Plot State parameters of the newest state of the batch
if plot_state
    if separate_figures
        figure('name','ConFusion Analysis - State');
        rows = ceil(sqrt(plot_state * num_state_params));
        columns = ceil(plot_state * num_state_params / rows);
    end

    num_states = size(confusion_data.batches, 2);
    states_new = zeros(1 + num_state_params_ext, num_states);
    for i=1:num_states
        states_new(1, i) = confusion_data.batches{i}.time(end); 
        param_index = 2;   
        for j=1:num_state_params  
            states_new(param_index:param_index+state_lengths(j)-1, i) = confusion_data.batches{i}.(state_names{j})(end, :);
            param_index = param_index + state_lengths(j);
        end
    end

    param_index = 2;
    for i=1:num_state_params
        axes(tot_plot_index) = subplot(rows, columns, page_tot_plot_index); hold on; grid on;
        tot_plot_index = tot_plot_index + 1;
        page_tot_plot_index = page_tot_plot_index + 1;
        for j=1:state_lengths(i)
            plot(states_new(1,:), states_new(param_index,:));
            param_index = param_index + 1;
        end
        ylabel(state_names{i}, 'interpreter', 'none');
    end
end

% Plot Static parameters
if plot_static_parameters
    if separate_figures
        figure('name','ConFusion Analysis - Static Parameters');
        rows = ceil(sqrt(plot_static_parameters * num_static_params));
        columns = ceil(plot_static_parameters * num_static_params / rows);
        page_tot_plot_index = 1;
    end

    for i=1:num_static_params
        axes(tot_plot_index) = subplot(rows, columns, page_tot_plot_index); hold on; grid on;
        tot_plot_index = tot_plot_index + 1;
        page_tot_plot_index = page_tot_plot_index + 1;
        param_index = 1;
        for j=1:size(confusion_data.staticParameters.(static_param_names{i}), 2)-1
            plot(confusion_data.staticParameters.(static_param_names{i})(:, 1), ...
                confusion_data.staticParameters.(static_param_names{i})(:, 1+param_index));
            param_index = param_index + 1;
        end
        ylabel(static_param_names(i), 'interpreter', 'none');
    end
end

% Plot the residual squared norms
if plot_residuals
    if separate_figures
        figure('name','ConFusion Analysis - Residuals');
        rows = ceil(sqrt(plot_residuals * 3));
        columns = ceil(plot_residuals * 3 / rows);
        page_tot_plot_index = 1;
    end
    
    % Prior residuals
    prior_residuals = zeros(2, size(confusion_data.priorResiduals, 1), 1);
    for i=1:size(confusion_data.priorResiduals, 1)
        prior_residuals(1, i) = confusion_data.priorResiduals(i, 1);
        prior_residuals(2, i) = confusion_data.priorResiduals(i, 2:end) * confusion_data.priorResiduals(i, 2:end)';
    end

    axes(tot_plot_index) = subplot(rows, columns, page_tot_plot_index); hold on; grid on;
    tot_plot_index = tot_plot_index + 1;
    page_tot_plot_index = page_tot_plot_index + 1;
    plot(states_new(1,:), prior_residuals(2, :));
    ylabel('e\_prior');
    
    % Process residuals of the newest states of the batches
    process_residuals = zeros(1 + num_process_sensors, size(confusion_data.processResidualBatches, 2));
    for i=1:size(confusion_data.processResidualBatches, 2)
        t_latest_state = confusion_data.processResidualBatches{i}(end, 1);
        process_residuals(1, i) = t_latest_state;
        for j=1:num_process_sensors
            % Look for a residual for each sensor at the current time
            for k=size(confusion_data.processResidualBatches{i},1):-1:1
                if confusion_data.processResidualBatches{i}(k, 1) ~= t_latest_state
                    % Residual not found for this sensor at this time
                    break;
                else
                    if confusion_data.processResidualBatches{i}(k, 2) == j-1
                        process_residuals(j+1, i) = process_residuals(j+1, i) + confusion_data.processResidualBatches{i}(k, 3:end) * confusion_data.processResidualBatches{i}(k, 3:end)';
                    end
                end
            end         
        end
    end
    
    axes(tot_plot_index) = subplot(rows, columns, page_tot_plot_index); hold on; grid on;
    tot_plot_index = tot_plot_index + 1;
    page_tot_plot_index = page_tot_plot_index + 1;
    for j=1:num_process_sensors
        plot(process_residuals(1, :), process_residuals(1+j, :));
    end
    ylabel('e\_process');
    
    % Update residuals of the newest states of the batches
    update_residuals = zeros(1 + num_update_sensors, size(confusion_data.updateResidualBatches, 2));
    for i=1:size(confusion_data.updateResidualBatches, 2)
        t_latest_state = confusion_data.updateResidualBatches{i}(end, 1);
        update_residuals(1, i) = t_latest_state;
        for j=1:num_update_sensors
            % Look for a residual for each sensor at the current time
            for k=size(confusion_data.updateResidualBatches{i},1):-1:1
                if confusion_data.updateResidualBatches{i}(k, 1) ~= t_latest_state
                    % Residual not found for this sensor at this time
                    break;
                else
                    if confusion_data.updateResidualBatches{i}(k, 2) == j-1
                        update_residuals(j+1, i) = update_residuals(j+1, i) + confusion_data.updateResidualBatches{i}(k, 3:end) * confusion_data.updateResidualBatches{i}(k, 3:end)';
                    end
                end
            end
        end
    end
    
    axes(tot_plot_index) = subplot(rows, columns, page_tot_plot_index); hold on; grid on;
    tot_plot_index = tot_plot_index + 1;
    page_tot_plot_index = page_tot_plot_index + 1;
    for j=1:num_update_sensors
        plot(update_residuals(1, :), update_residuals(1+j, :));
    end
    ylabel('e\_update');
end

% Plot user data. todo Only doing this in a separate plot for now.
if plot_user_data
    figure('name','ConFusion Analysis - User Data'); hold on;
    for i=1:size(confusion_data.userData, 2)
        plot(1:size(confusion_data.userData, 1),confusion_data.userData(:,i));
    end
end

linkaxes(axes,'x');

end

