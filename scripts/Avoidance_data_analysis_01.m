%% FILE SUMMARY
% Purpose: Segment and visualize avoidance trajectories for one selected trial condition.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Signal Processing Toolbox; Curve Fitting Toolbox.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: CSV logs in data/magnetic_data/avoidance_exp (added to path via setup_project_paths).
% Outputs: Time-series plots, trial-resampled 3D trajectories, and obstacle-region overlays.
% Run Notes: Set analysisType to select condition (real/virtual, left/center/right magnet).


%% NOTE for CODE
% This code has the basic structure to segment trials in collision
% avoidance experiments.
% Change the variable "analysisType" from 1 to 6 to view different EE
% trajectories visualized with the avoidance region, waypoints and the 
% magnet.
%
% This code generates panels b,c and d of First Results Figure related to
% collision avoidance. In detail:
% - analysisType = 1 generates panel b
% - analysisType = 2 generates panel c
% - analysisType = 3 generates panel d
%
% analysisType = 4,5,6 show the same plots when the tracking algorithm was
% not active and the robot was controlled knowing the exact magnet
% location.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath(genpath("data/session_01"))
    end 
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    %% Operations over path
    currentPath = pwd;
    cd ..
    addpath(genpath("data/session_01"))
    cd(currentPath)
end

addpath("dependencies")

%% Data Loading

% Number to switch the file analysed
analysisType = 1;

% Target points in the workspace for the robot end-effector.
waypoints = [0.466457,   0.45,  0.40
             0.466457,  -0.45,  0.40];
magnetPosition = [];

switch analysisType
    case 1 % Magnetic Tracking Active - Magnet on the Left side
        filename = "Avoidance_REAL1_exp_09_07_15_57_32.csv";
        magGT = [0.466 -0.215 0.3265 0 1 0];
    case 2 % Magnetic Tracking Active - Magnet on the Center
        filename = "Avoidance_REAL_2_exp_09_07_17_17_08.csv";
        magGT = [0.466 -0.01 0.3265 0 1 0];
    case 3 % Magnetic Tracking Active - Magnet on the Right side
        filename = "Avoidance_REAL_3_exp_09_07_18_19_40.csv";
        magGT = [0.466 0.194 0.3265 0 1 0];
    case 4 % Magnetic Tracking NOT Active - Magnet on the Left side
        filename = "Avoidance_VIRTUAL1_exp_09_07_16_11_46.csv";
        magGT = [0.466 -0.215 0.3265 0 1 0];
    case 5 % Magnetic Tracking NOT Active - Magnet on the Center
        filename = "Avoidance_VIRTUAL2_exp_09_07_17_02_15.csv";
        magGT = [0.466 -0.01 0.3265 0 1 0];
    case 6 % Magnetic Tracking NOT Active - Magnet on the Right side
        filename = "Avoidance_VIRTUAL_3_exp_09_07_18_07_01.csv";
        magGT = [0.466 0.194 0.3265 0 1 0];
end

%% DATA EXTRACTIONS FROM FILE

% custom function to read/import csv data exported in c++ (see at the end of the file)
dataTable = import_CSV_Table(filename);

time = dataTable.time;

time        = time-time(1); % reset time start
magPose     = dataTable{:,2:7};
magField    = dataTable{:,17:end};
EE_position = dataTable{:,8:10};
EE_velocity = dataTable{:,11:13};

%% Data Display

isOutliers = magPose(:,end) == -1;
magPose(isOutliers,:) = nan;

isZero = EE_position(:,1) == 0;
EE_position(isZero,:) = nan;
EE_velocity(isZero,:) = nan;

[pks,locs] = findpeaks(EE_position(:,2),"MinPeakHeight",max(EE_position(:,2))/2);

figure("Units","normalized","Position",[0.3 0.3 0.5 0.3])
plot(time,magPose)
xlabel("Time (s)","FontSize",10,"Interpreter","latex")
legend(["$x_M$" "$y_M$" "$z_M$" "$m_x$" "$m_y$" "$m_z$"],...
       "FontSize",10,"Interpreter","latex","Location","eastoutside")

figure
plot(time,magField)
ylabel("Magnetic Field Strength ($\mu T$)","FontSize",10,"Interpreter","latex")
xlabel("Time (s)","FontSize",10,"Interpreter","latex")
legend(["$B_{1x}$", "$B_{1y}$", "$B_{1z}$", ...
        "$B_{2x}$", "$B_{2y}$", "$B_{2z}$", ...
        "$B_{3x}$", "$B_{3y}$", "$B_{3z}$", ...
        "$B_{4x}$", "$B_{4y}$", "$B_{4z}$"],...
       "FontSize",10,"Interpreter","latex","Location","eastoutside")

figure
subplot(211)
hold on
plot(time,EE_position)
plot(time(locs),pks,'k*')
hold off
ylabel("EE position (m)","FontSize",10,"Interpreter","latex")
xlabel("Time (s)","FontSize",10,"Interpreter","latex")
subplot(212)
plot(time,EE_velocity)
ylabel("EE velocity (m/s)","FontSize",10,"Interpreter","latex")
xlabel("Time (s)","FontSize",10,"Interpreter","latex")


%% TRIALS SEGMENTATION

numSamplesInterval = diff(locs);
maxNumSamplesInterval = max(numSamplesInterval);
normalizedTime = linspace(0,1,maxNumSamplesInterval);
nTrials = numel(locs)-1;
nSamplFiltering = 100;

resampledTrials_x = zeros(maxNumSamplesInterval,nTrials-1);
resampledTrials_y = resampledTrials_x;
resampledTrials_z = resampledTrials_x;
xq = normalizedTime';

for i = 2:nTrials
    x = time(locs(i):locs(i+1));
    x = x-x(1);
    x = x/max(x);

    % for EEx
    y = EE_position(locs(i):locs(i+1),1);
    resampledTrials_x(:,i-1) = smooth(pchip(x,y,xq),nSamplFiltering);

    % for EEy
    y = EE_position(locs(i):locs(i+1),2);
    resampledTrials_y(:,i-1) = smooth(pchip(x,y,xq),nSamplFiltering);

    % for EEz
    y = EE_position(locs(i):locs(i+1),3);
    resampledTrials_z(:,i-1) = smooth(pchip(x,y,xq),nSamplFiltering);
end

figure
subplot(131)
plot(normalizedTime,resampledTrials_x);
subplot(132)
plot(normalizedTime,resampledTrials_y);
subplot(133)
plot(normalizedTime,resampledTrials_z);

R_avoid = 0.18;
angles = linspace(0,2*pi,100);

[X,Y,Z] = sphere(100);
X = X*R_avoid + magGT(:,1);
Y = Y*R_avoid + magGT(:,2);
Z = Z*R_avoid + magGT(:,3);


figure
hold on
for i = 1:nTrials-1
    plot3(resampledTrials_x(:,i),resampledTrials_y(:,i),resampledTrials_z(:,i))
end
scatter3(magGT(1),magGT(2),magGT(3),8,'filled','k')
surf(X,Y,Z,"EdgeColor","none","FaceColor",[0 0.4470 0.7410])
alpha 0.1
scatter3(waypoints(:,1),waypoints(:,2),waypoints(:,3),35,'filled','k')
hold off
% axis([0.4 0.5 -0.5 0.5 0 1])
axis equal
axis off
view(40,25)

figure
hold on
for i = 1:nTrials-1
    plot(resampledTrials_y(:,i),resampledTrials_z(:,i))
end
scatter(magGT(2),magGT(3),8,'filled','k')
plot(magGT(:,2)+R_avoid*cos(angles),magGT(:,3)+R_avoid*sin(angles),'k--')
scatter(waypoints(:,2),waypoints(:,3),35,'filled','k')
axis([-0.4 0.4 0 0.75])
axis equal

%% Auxiliary Function

function data = import_CSV_Table(filename, dataLines)
% Auto-generated by MATLAB on 31-Oct-2024 16:52:49

% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 28);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "mag_pos_x", "mag_pos_y", "mag_pos_z", "mag_orien_x", "mag_orien_y", "mag_orien_z", "ee_x", "ee_y", "ee_z", "v_ee_x", "v_ee_y", "v_ee_z", "F_ee_x", "F_ee_y", "F_ee_z", "s1_x", "s1_y", "s1_z", "s2_x", "s2_y", "s2_z", "s3_x", "s3_y", "s3_z", "s4_x", "s4_y", "s4_z"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);

end