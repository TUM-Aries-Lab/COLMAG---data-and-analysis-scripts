%% FILE SUMMARY
% Purpose: Compare completion-time behavior between guidance modalities across subjects.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; Statistics and Machine Learning Toolbox; local package dependencies/dabarplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: Participant CSV files in data/magnetic_data/experiments_mod_1 and experiments_mod_2.
% Outputs: Condition-wise time plots, summary bars, and paired statistical test results.
% Run Notes: File lists are manually curated to enforce participant alignment.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath(genpath("data/session_01"))
        addpath("dependencies")
    end 
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % Operations over path
    currentPath = pwd;
    cd ..
    addpath(genpath("data/session_01"))
    cd(currentPath)
    addpath("../dependencies")
end

%%
% Participant S02 is excluded from files/files2: data_em1_S02.csv and
% data_em2_S02.csv are byte-identical duplicates of a single recording
% (confirmed via checksum), so no independent Mode-2 measurement exists
% for S02, and no backup of the original recording is available.

files = ["data_em1_S01.csv"
         "data_em1_S03.csv"
         "data_em1_S04.csv"
         "data_em1_S05.csv"
         "data_em1_S06.csv"
         "data_em1_S07.csv"
         "data_em1_S08.csv"
         "data_em1_S09.csv"];
files2 = ["data_em2_S01.csv"
          "data_em2_S03.csv"
          "data_em2_S04.csv"
          "data_em2_S05.csv"
          "data_em2_S06.csv"
          "data_em2_S07.csv"
          "data_em2_S08.csv"
          "data_em2_S09.csv"];

for i = 1:size(files,1)
    data = importGuidanceFile(files(i));

    time = data(:,1)-data(1,1);
    EEpos = data(:,8:10);

    struct.time(i) = time(end);
    figure
    plot(time,EEpos(:,3))
end

for i = 1:size(files2,1)
    data = importGuidanceFile(files2(i));

    time = data(:,1)-data(1,1);
    EEpos = data(:,8:10);

    struct.time2(i) = time(end);
    figure
    plot(time,EEpos(:,3))
end

times = struct.time;

figure
hold on
plot(struct.time,"o")
plot(struct.time2,"x")

figure
h = dabarplot([struct.time;struct.time2]');
overlaySubjectScatter([struct.time;struct.time2]', ones(8,1), h.gpos);

% Completion times are within-participant (same 8 participants in both
% modes), so the paired Wilcoxon signed-rank test (signrank) is used
% instead of the unpaired ranksum/Mann-Whitney test.
try
    if(test)
        [p,h,stats] = signrank(struct.time',struct.time2');
    end
catch exception
    disp("****** WILCOXON SIGNED-RANK (paired) on task completion time for MOD1 vs MOD2 ******")
    [p,h,stats] = signrank(struct.time',struct.time2')
    disp("****** ------------------------------ ******")
end

%%

function overlaySubjectScatter(data, group_inx, gpos)
% Per-subject colored scatter points with connecting lines overlaid on dabarplot.
% Within each group, same-subject markers across conditions are connected.
% For single-condition data, same-subject markers are connected between groups.
% n_subjects = 8: participant S02 is excluded throughout this script
% (duplicate Mode-1/Mode-2 recording, see note at top of file).
n_subjects = 8;
subj_colors = lines(n_subjects);
markers     = {'o','s','d','^','v','>','<','p'};

data   = double(data);
if isvector(data), data = data(:); end
groups     = unique(group_inx(:));
num_groups = numel(groups);
num_conds  = size(data, 2);
gpos       = reshape(gpos, num_groups, num_conds);

% Jitter scale: 20% of the minimum inter-bar gap, clamped to [0.02, 0.06]
all_x = sort(gpos(:));
if numel(all_x) > 1
    jitter_scale = min(diff(all_x)) * 0.20;
    jitter_scale = min(max(jitter_scale, 0.02), 0.06);
else
    jitter_scale = 0.04;
end

% Pre-generate per-subject offsets for each group (same offset across all
% conditions so within-group connecting lines stay parallel and don't cross)
x_offs = (rand(n_subjects, num_groups) - 0.5) * 2 * jitter_scale;

for g = 1:num_groups
    mask = group_inx == groups(g);
    grp  = data(mask, :);
    for i = 1:n_subjects
        col = subj_colors(i,:);
        xp  = gpos(g,:) + x_offs(i,g);
        yp  = grp(i,:);
        if num_conds > 1
            lh = plot(xp, yp, '-', 'Color', col, 'LineWidth', 0.8, 'HandleVisibility', 'off');
            lh.Color(4) = 0.35;
            hold on;
        end
        scatter(xp, yp, 30, col, markers{i}, 'filled', ...
            'MarkerFaceAlpha', 0.7, 'MarkerEdgeColor', 'k', 'LineWidth', 0.5, ...
            'HandleVisibility', 'off');
        hold on;
    end
end

% For single-condition plots connect the same subject between the two groups
if num_conds == 1 && num_groups == 2
    d1 = data(group_inx == groups(1), :);
    d2 = data(group_inx == groups(2), :);
    for i = 1:n_subjects
        col = subj_colors(i,:);
        lh  = plot([gpos(1)+x_offs(i,1)  gpos(2)+x_offs(i,2)], [d1(i) d2(i)], '-', ...
            'Color', col, 'LineWidth', 0.8, 'HandleVisibility', 'off');
        lh.Color(4) = 0.35;
        hold on;
    end
end
end

function data = importGuidanceFile(filename, dataLines)
%IMPORTFILE Import data from a text file
%  CHIARAEXP0908121702 = IMPORTFILE(FILENAME) reads data from text file
%  FILENAME for the default selection.  Returns the numeric data.
%
%  CHIARAEXP0908121702 = IMPORTFILE(FILE, DATALINES) reads data for the
%  specified row interval(s) of text file FILENAME. Specify DATALINES as
%  a positive scalar integer or a N-by-2 array of positive scalar
%  integers for dis-contiguous row intervals.
%
%  Example:
%  CHIARAexp0908121702 = importfile("C:\Users\Federico\OneDrive\Lavoro ARIES\06 - Phd &Internships & Thesis\Giuliano Giampietro\Data Analysis\magnetic_data\experiments_mod_1\CHIARA_exp_09_08_12_17_02.csv", [2, Inf]);
%
%  See also READTABLE.
%
% Auto-generated by MATLAB on 19-Jun-2025 23:15:04

%% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

%% Set up the Import Options and import the data
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

%% Convert to output type
data = table2array(data);
end

