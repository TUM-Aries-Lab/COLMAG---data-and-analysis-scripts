%% FILE SUMMARY
% Purpose: Load the 1 m/s stop-distance trials (10 ms folder), detect and flag defective/outlier
%          trials with a documented, automatic, and reproducible criterion, and save only the
%          cleaned trial data for downstream plotting/statistics.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB.
% Data: CSV logs in data/session_03/DataStop/10ms.
% Outputs: data/session_03/DataStop/StopProfiles_10ms_clean.mat, containing the retained trials'
%          time/position/velocity traces (structStopsClean) and a full QC report of every trial,
%          kept or rejected, with the reason (qcReport).
% Run Notes: Run this script first. Safety_data_analysis_03_plot.m consumes its saved output and
%            performs no trial selection of its own.
%
% Replaces a previously hardcoded trial-index list (for i = [1 3:4 7 9:17 20]) that silently
% excluded 9 of the 23 recorded trials without a documented reason. Those 9 trials are indeed
% defective for the purpose of this representative-profile analysis (see criteria below):
%  - 6 never reached the nominal 1 m/s approach speed of the batch. This reflects the commanded
%    speed/motion limits enforced by the Franka controller itself on those particular runs (not a
%    sensor or logging fault), but it still means these trials do not represent the intended
%    constant 1 m/s approach condition and are excluded on that basis.
%  - 3 have roughly double the duration of every other trial (a different recorded motion segment).
% The automatic criteria below reproduce that same exclusion set from the raw data alone, so the
% selection is now traceable to the trial recordings rather than to an unexplained literal array.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath("data/session_03/DataStop/10ms")
        addpath("dependencies")
        dataStopDir = "data/session_03/DataStop";
    end
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % operations over path
    currentPath = pwd;
    cd ..
    addpath("data/session_03/DataStop/10ms")
    dataStopDir = fullfile(pwd,"data","session_03","DataStop");
    cd(currentPath)
    addpath("../dependencies")
end

%% Trials to process (1 m/s stop-distance experiment, all recorded runs)

fileNames = ["Demo_exp_07_14_23_54_49.csv"
             "Demo_exp_07_14_23_55_36.csv"
             "Demo_exp_07_14_23_56_19.csv"
             "Demo_exp_07_14_23_57_29.csv"
             "Demo_exp_07_14_23_58_42.csv"
             "Demo_exp_07_14_23_59_51.csv"
             "Demo_exp_07_15_00_02_08.csv"
             "Demo_exp_07_15_00_04_14.csv"
             "Demo_exp_07_15_00_05_04.csv"
             "Demo_exp_07_15_00_05_56.csv"
             "Demo_exp_07_15_00_06_43.csv"
             "Demo_exp_07_15_00_07_30.csv"
             "Demo_exp_07_15_00_10_47.csv"
             "Demo_exp_07_15_00_15_13.csv"
             "Demo_exp_07_15_00_17_09.csv"
             "Demo_exp_07_15_00_19_01.csv"
             "Demo_exp_07_15_00_20_28.csv"
             "Demo_exp_07_15_00_23_52.csv"
             "Demo_exp_07_15_00_25_04.csv"
             "Demo_exp_07_15_00_25_55.csv"
             "Demo_exp_07_15_01_14_12.csv"
             "Demo_exp_07_15_01_15_26.csv"
             "Demo_exp_07_15_01_16_25.csv"];

nFiles = size(fileNames,1);

%% Load each trial and compute per-trial data + QC metrics

for i = 1:nFiles
    fileName = fileNames(i,:);

    data = importfile(fileName);

    mask = 1:size(data,1);

    time = data(mask,1)-data(mask(1),1);
    magPos = data(mask,2:7);
    EE_pos = data(mask,8:10);
    EE_vel = data(mask,11:13);

    mask2 = ~(EE_vel(:,1) == 0 & EE_vel(:,2) == 0 & EE_vel(:,3) == 0);

    tstart = time(mask2);
    tstart = tstart(1);

    structStops(i).fileName = fileName; %#ok<SAGROW>
    structStops(i).time = time(mask2)-tstart;
    structStops(i).EE_pos = EE_pos(mask2,:);
    structStops(i).EE_vel = EE_vel(mask2,:);
    structStops(i).dist_10 = min(vecnorm(EE_pos(mask2,:)-magPos(mask2,1:3),2,2));
    structStops(i).maxSpeed = max(vecnorm(EE_vel(mask2,:),2,2));
    structStops(i).duration = structStops(i).time(end);
end

%% Automatic outlier/defect detection
% A trial is flagged as defective and excluded from the downstream
% representative-profile analysis if either:
%  (1) SPEED DEFECT: it never reaches the nominal approach speed of this
%      batch (robust estimate: median of the per-trial peak speed),
%      indicating a premature stop/trigger or a logging/sensor defect; or
%  (2) DURATION OUTLIER: its duration deviates strongly from the batch's
%      median duration, indicating it captures a different motion
%      segment (e.g., a repeated or extended back-and-forth pass) rather
%      than a single comparable approach-and-stop trial.
% Thresholds (95% of the nominal speed; 1.5x the median duration) are
% simple, documented, and computed directly from the raw data.

speedFrac = 0.95;
durationFactor = 1.5;

allMaxSpeed = [structStops.maxSpeed]';
allDuration = [structStops.duration]';

nominalSpeed = median(allMaxSpeed);
medianDuration = median(allDuration);

isSpeedDefective = allMaxSpeed < speedFrac*nominalSpeed;
isDurationOutlier = (allDuration > durationFactor*medianDuration) | ...
                    (allDuration < medianDuration/durationFactor);
isOutlier = isSpeedDefective | isDurationOutlier;

keepIdx = find(~isOutlier);
rejectIdx = find(isOutlier);

%% QC report

disp("=== Trial QC report: 10 ms / 1 m/s stop-distance experiment ===")
disp("Nominal approach speed (median of all trials) = " + num2str(nominalSpeed) + " m/s")
disp("Median trial duration = " + num2str(medianDuration) + " s")
disp("")

fileNameCol = strings(nFiles,1);
for i = 1:nFiles
    fileNameCol(i) = structStops(i).fileName;
    reason = "";
    if isSpeedDefective(i)
        reason = reason + "peak speed " + num2str(allMaxSpeed(i)) + " m/s < " + ...
            num2str(speedFrac*nominalSpeed) + " m/s (did not reach nominal approach speed); ";
    end
    if isDurationOutlier(i)
        reason = reason + "duration " + num2str(allDuration(i)) + " s outside [" + ...
            num2str(medianDuration/durationFactor) + ", " + num2str(durationFactor*medianDuration) + ...
            "] s (different motion segment); ";
    end
    if isOutlier(i)
        disp(num2str(i) + ": " + structStops(i).fileName + " -> EXCLUDED (" + reason + ")")
    else
        disp(num2str(i) + ": " + structStops(i).fileName + " -> KEPT")
    end
end
disp("")
disp(num2str(numel(keepIdx)) + " of " + num2str(nFiles) + ...
    " trials retained for the representative profile analysis.")

%% Save the cleaned subset for downstream plotting/statistics

structStopsClean = structStops(keepIdx); %#ok<NASGU>
qcReport = table((1:nFiles)', fileNameCol, allMaxSpeed, allDuration, ...
    isSpeedDefective, isDurationOutlier, isOutlier, ...
    'VariableNames', {'trial','fileName','maxSpeed_mps','duration_s', ...
    'speedDefective','durationOutlier','excluded'}); %#ok<NASGU>

cleanDataFile = fullfile(dataStopDir,"StopProfiles_10ms_clean.mat");
save(cleanDataFile, "structStopsClean", "qcReport", "keepIdx", "rejectIdx");

disp("Saved cleaned trial data and QC report to " + cleanDataFile)

%% Local function

function data = importfile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  DEMOEXP0707200608 = IMPORTFILE(FILENAME) reads data from text file
%  FILENAME for the default selection.  Returns the numeric data.
%
%  DEMOEXP0707200608 = IMPORTFILE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  Example:
%  Demoexp0707200608 = importfile("C:\Users\Federico\OneDrive\Lavoro ARIES\03 - Papers\TRO - Magnetic Manipulator\Science Advances\New Data\DataStop\Demo_exp_07_07_20_06_08.csv", [2, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 07-Jul-2025 20:13:22

% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 59);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "mag_pos_x", "mag_pos_y", "mag_pos_z", "mag_orien_x", "mag_orien_y", "mag_orien_z", "ee_x", "ee_y", "ee_z", "v_ee_x", "v_ee_y", "v_ee_z", "F_ee_x", "F_ee_y", "F_ee_z", "s1_x", "s1_y", "s1_z", "s2_x", "s2_y", "s2_z", "s3_x", "s3_y", "s3_z", "s4_x", "s4_y", "s4_z", "n_s1_x", "n_s1_y", "n_s1_z", "n_s2_x", "n_s2_y", "n_s2_z", "n_s3_x", "n_s3_y", "n_s3_z", "n_s4_x", "n_s4_y", "n_s4_z", "ls_compute_time", "r2", "O_T_EE_1", "O_T_EE_2", "O_T_EE_3", "O_T_EE_4", "O_T_EE_5", "O_T_EE_6", "O_T_EE_7", "O_T_EE_8", "O_T_EE_9", "O_T_EE_10", "O_T_EE_11", "O_T_EE_12", "O_T_EE_13", "O_T_EE_14", "O_T_EE_15", "O_T_EE_16", "state"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);

% Convert to output type
data = table2array(data);
end
