confusion_data = plotConFusionData('../../confusion/data/tagtracker_log.txt',true,true,true,false,true);

% Build a matrix of the estimated poses (end of batch)
num_states = size(confusion_data.batches, 2);
est_poses = zeros(num_states, 8);
for i=1:num_states
    est_poses(i, 1) = confusion_data.batches{i}.time(end); 
    for j=1:3  
        est_poses(i, 1+j) = confusion_data.batches{i}.t_w_i(end, j);
    end
    for j=1:4
        est_poses(i, 4+j) = confusion_data.batches{i}.q_w_i(end, j);
    end
end

% Compute the error between the estimated and measured poses
pose_errors = zeros(num_states, 7);
pose_errors(:,1) = est_poses(:,1);
pose_errors(:,2:4) = est_poses(:,2:4) - confusion_data.userData(:,2:4);
for i=1:num_states
    pose_errors(i,5:7) = QuatDistanceVec(est_poses(i,5:8),confusion_data.userData(i,5:8));
end

% Plot the raw measured and estimated IMU poses
figure();
axes = zeros(4);
axes(1) = subplot(2,2,1); grid on; hold on;
for i=1:3
    plot(est_poses(:,1),est_poses(:,i+1));
    plot(confusion_data.userData(:,1),confusion_data.userData(:,i+1),'--');
end
ylabel('Position [m]');

axes(2) = subplot(2,2,3); grid on; hold on;
for i=1:4
    plot(est_poses(:,1),est_poses(:,i+4));
    plot(confusion_data.userData(:,1),confusion_data.userData(:,i+4),'--');
end
ylabel('Orientation [quat]');

axes(3) = subplot(2,2,2); grid on; hold on;
for i=1:3
    plot(pose_errors(:,1),pose_errors(:,i+1));
end
ylabel('Pos Error [m]');

axes(4) = subplot(2,2,4); grid on; hold on;
for i=1:3
    plot(pose_errors(:,1),pose_errors(:,i+4));
end
ylabel('Rot Error [rad]');

linkaxes(axes,'x');