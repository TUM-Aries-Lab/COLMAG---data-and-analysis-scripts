%% FILE SUMMARY
% Purpose: Analyze stop behavior from ROS bag batches and compare stopping distances.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; ROS Toolbox; Signal Processing Toolbox; local package dependencies/dabarplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: ROS bag files in data/nuove_prove_stop/test_safety_speed.
% Outputs: Filtered EE trajectories, velocity profiles, and stop-distance summary plots.
% Run Notes: Includes manual reference-point selection with ginput for selected trials.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        folderPath = "data/session_04";
        addpath(genpath(folderPath))
        bag_files = dir("data/session_04/**/*.bag");
    end
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % operations over path
    currentPath = pwd;
    cd ..
    folderPath = "data/session_04";
    bag_files = dir("data/session_04/**/*.bag");
    addpath(genpath(folderPath))
    cd(currentPath)
end

addpath("dependencies")

%%

SelectPoints = false;

num_bags = length(bag_files);
data_struct = struct();

for k = 1:num_bags
    bag_name = bag_files(k).name;
    bag_folder = bag_files(k).folder;
    bag = rosbagreader(fullfile(bag_folder, bag_name));
    sel_EE = select(bag, "Topic", "/fr3_EE");
    msgs_EE = readMessages(sel_EE, "DataFormat", "struct");

    n = numel(msgs_EE);
    data_EE = zeros(n, 3);
    time = zeros(n, 1);

    for i = 1:n
        stamp = msgs_EE{i}.Header.Stamp;
        time(i) = double(stamp.Sec) + double(stamp.Nsec) * 1e-9;
        data_EE(i, 1) = msgs_EE{i}.Pose.Position.X;
        data_EE(i, 2) = msgs_EE{i}.Pose.Position.Y;
        data_EE(i, 3) = msgs_EE{i}.Pose.Position.Z;
    end

    time = time - time(1);

    %% Smooth the data
    dt = mean(diff(time));
    fs = round(1/dt);
    fc = 4; % cutoff frequency in Hz, adjust as needed
    [b,a] = butter(4, fc/(fs/2));
    data_smooth_EE = data_EE;
    for i = 1:3
        data_smooth_EE(:,i) = filtfilt(b,a, data_EE(:,i));
    end

    %% Calculate velocity
    vel = zeros(n-1, 3);
    for i = 1:3
        vel(:, i) = diff(data_smooth_EE(:, i)) ./ diff(time);
    end
    vel_norm = sqrt(sum(vel.^2, 2));
    time_vel = time(1:end-1) + diff(time)/2;

    data_struct(k).folder = bag_folder;
    data_struct(k).name = bag_name;
    data_struct(k).time = time;
    data_struct(k).position = data_smooth_EE;
    data_struct(k).velocity = vel_norm;
end

%%

ref_stop_motion = data_struct(13).position;
ref_stop_motion = ref_stop_motion(end,:);

%%

%select the points via ginput for the analysis

if (SelectPoints)

    selected_points = zeros(11, 3);
    for i = 1:11
        time_i = data_struct(i).time;
        pos_i = data_struct(i).position;
        figure
        subplot(311)
        plot(time_i, pos_i(:,1))
        title(['X position for i = ', num2str(i)])
        subplot(312)
        plot(time_i, pos_i(:,2))
        title('Y position')
        subplot(313)
        plot(time_i, pos_i(:,3))
        title('Z position')
        [x, ~] = ginput(1);  % select x (time)
        % find closest index
        [~, idx] = min(abs(time_i - x));
        selected_points(i, :) = pos_i(idx, :);
    end
    distances = vecnorm(selected_points([1:2 4:9 11],:)-ref_stop_motion,2,2);

else

    distances = [0.290129129
                0.309632188
                0.300395832
                0.340405158
                0.350625842
                0.346462757
                0.299686103
                0.302869596
                0.300949808
                0.293549716
                ];
end

%%

figure("Units","normalized","Position",[0.3 0.3 0.15 0.3])
hold on
h = dabarplot(distances,'errorbars','SD',...
    'barspacing',0.8);
scatter(ones(size(distances)),distances,25,"filled","k")