%% FILE SUMMARY
% Purpose: Analyze guidance user-study results (errors, NASA-TLX, trajectory metrics).
% Last Updated: 2026-08-13.
% Dependencies: MATLAB; Statistics and Machine Learning Toolbox; local packages dependencies/dabarplot.m and dependencies/daviolinplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: Embedded questionnaire arrays plus CSV logs in data/magnetic_data.
% Outputs: Comparative bar/violin plots and statistical comparisons across guidance modes.
% Run Notes: Keep participant ordering consistent when editing embedded arrays.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath("data/session_01/experiments_mod_1")
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
    addpath("data/session_01/experiments_mod_1")
    cd(currentPath)
    addpath("../dependencies")
end

%% Data Collected via questionaires and monitoring results in pick and place
% Participant S02 is excluded: the raw Mode-1 and Mode-2 recordings for
% S02 (data_em1_S02.csv, data_em2_S02.csv) are byte-identical duplicates
% of a single recording (confirmed via checksum), so no independent Mode-2
% measurement exists for this participant. No backup of the original,
% non-duplicated recording is available. All within-participant (n=8)
% comparisons below therefore exclude S02 rather than filter it at
% runtime.

Errors_mod1 = [ 0	0	0    % S1
    0	0	2    % S3
    0	3	4    % S4
    0	0	1    % S5
    4	1	3    % S6
    0	1	2    % S7
    1	1	4    % S8
    0	4	2 ]; % S9

Errors_mod2 = [ 1   1	4    % S1
    0	1	1    % S3
    1	3	3    % S4
    1	0	5    % S5
    0	1	4    % S6
    0	4	3    % S7
    1	2	3    % S8
    0	0	5 ]; % S9

% Raw tick-mark index (not a score) at which the participant marked each
% of the 6 NASA-TLX subscale questions on the paper form. The form has 21
% gradations of 5, from 0 to 100, so each index step is worth 5 points;
% converted to the standard 0-100 subscale score below. Columns follow the
% standard NASA-TLX order: Mental Demand, Physical Demand, Temporal
% Demand, Performance, Effort, Frustration.
NASA_mod1_raw = [   4 5 1 2 3 2     % S1
    7 3 1 3 13 2    % S3
    5 2 2 2 2 1     % S4
    4 2 4 2 6 1     % S5
    9 6 10 4 4 6    % S6
    5 1 1 12 7 2    % S7
    7 2 2 6 10 4    % S8
    4 4 1 9 4 1];   % S9

NASA_mod2_raw = [   7 8 2 2 2 2     % S1
    2 2 2 3 1 1     % S3
    6 4 4 3 2 2     % S4
    12 13 10 9 10 8 % S5
    6 2 4 6 2 3     % S6
    3 1 1 6 2 2     % S7
    4 2 5 2 2 2     % S8
    5 5 2 3 5 3];   % S9

% Tick-mark index -> standard 0-100 NASA-TLX subscale score.
NASA_mod1 = NASA_mod1_raw * 5;
NASA_mod2 = NASA_mod2_raw * 5;

NASA_dims = {'Mental Demand','Physical Demand','Temporal Demand','Performance','Effort','Frustration'};

nSubj = size(NASA_mod1,1); % 8, after excluding S02

%%
% All Mode-1 vs Mode-2 comparisons below are within-participant (same
% nSubj participants performed both modes), so paired tests (signrank)
% are used throughout instead of the unpaired ranksum/Mann-Whitney test.

NASA = [NASA_mod1; NASA_mod2];
% Unweighted (Raw TLX) overall score: mean of the 6 (0-100) subscale scores.
% No pairwise-comparison weights were collected, so the classic weighted
% NASA-TLX score cannot be computed.
NASA_score1 = mean(NASA_mod1,2);
NASA_score2 = mean(NASA_mod2,2);
NASA_score = [NASA_score1; NASA_score2];
group_inx = [ones(1,nSubj) 2*ones(1,nSubj)]';

% Combined view for the standard NASA-TLX chart: the 6 subscales plus the
% overall score, side by side.
NASA_withTotal = [NASA_mod1 NASA_score1; NASA_mod2 NASA_score2];
NASA_labels = [NASA_dims, {'Overall'}];

Errors = [Errors_mod1; Errors_mod2];
group_errs = [ones(1,nSubj) 2*ones(1,nSubj)]';
% Total errors per participant (sum across the 3 hole sizes); reported as
% "total errors" in Fig. 8C.
Errors_Sum1 = sum(Errors_mod1,2);
Errors_Sum2 = sum(Errors_mod2,2);
Errors_Sum = [Errors_Sum1; Errors_Sum2];

% Participant-level aggregate across modes, per hole size: summing Mode 1
% and Mode 2 error counts for each participant gives one independent
% observation per participant (N=8). This is the correct unit of analysis
% for the hole-size Friedman test: stacking Errors_mod1 and Errors_mod2
% row-wise (as [Errors_mod1; Errors_mod2], N=16) would instead treat each
% participant's two modes as independent blocks, which they are not (same
% 8 participants performed both), understating uncertainty.
Errors_bySubject = Errors_mod1 + Errors_mod2;

try
    if(test)
        for j = 1:6
            [p,h,stats] = signrank(NASA_mod1(:,j),NASA_mod2(:,j));
        end

        figure
        hold on
        h = dabarplot(NASA_withTotal,'groups',group_inx,'errorbars','SE',...
            'barspacing',0.8,'xtlabels',NASA_labels);
        overlaySubjectScatter(NASA_withTotal, group_inx, h.gpos);
        ylabel('NASA-TLX score (0-100)')
        ylim([0 100])
        hold off

        figure
        hold on
        h3 = dabarplot(NASA_score,'groups',group_inx,'errorbars','SE',...
            'barspacing',0.8,'xtlabels',{'Mode 1','Mode 2'});
        overlaySubjectScatter(NASA_score, group_inx, h3.gpos);
        ylabel('Overall NASA-TLX score (0-100)')
        ylim([0 70])
        hold off

        [p,h,stats] = signrank(NASA_score1,NASA_score2);

        %%

        for j = 1:3
            [p,h,stats] = signrank(Errors_mod1(:,j),Errors_mod2(:,j));
        end

        [p,h,stats] = signrank(Errors_Sum1,Errors_Sum2);

        [pOmni1,tblOmni1,statsOmni1] = friedman(Errors_mod1);
        [posthoc1,means1,~,gnames1] = multcompare(statsOmni1,"CriticalValueType","bonferroni");

        [pOmni2,tblOmni2,statsOmni2] = friedman(Errors_mod2);
        [posthoc2,means2,~,gnames2] = multcompare(statsOmni2,"CriticalValueType","bonferroni");

    end
catch exception
    disp("****** WILCOXON SIGNED-RANK (paired) on the individual NASA-TLX dimensions for MOD1 vs MOD2 ******")
    for j = 1:6
        disp("dimension j = " + num2str(j))
        [p,h,stats] = signrank(NASA_mod1(:,j),NASA_mod2(:,j))
    end
    disp("****** ------------------------------ ******")
    disp("")

    figure
    hold on
    h = dabarplot(NASA_withTotal,'groups',group_inx,'errorbars','SE',...
        'barspacing',0.8,'xtlabels',NASA_labels,'legend',{'Mode 1','Mode 2'});
    overlaySubjectScatter(NASA_withTotal, group_inx, h.gpos);
    ylabel('NASA-TLX score (0-100)')
    ylim([0 100])
    hold off

    figure
    hold on
    h3 = dabarplot(NASA_score,'groups',group_inx,'errorbars','SE',...
        'barspacing',0.8,'xtlabels',{'Mode 1','Mode 2'});
    overlaySubjectScatter(NASA_score, group_inx, h3.gpos);
    ylabel('Overall NASA-TLX score (0-100)')
    ylim([0 100])
    hold off

    disp("****** WILCOXON SIGNED-RANK (paired) on the total (unweighted mean) NASA-TLX score for MOD1 vs MOD2, reported in Fig. 8E ******")
    [p,h,stats] = signrank(NASA_score1,NASA_score2)
    disp("****** ------------------------------ ******")
    disp("")

    %%

    disp("****** WILCOXON SIGNED-RANK (paired) on the individual error distribution for MOD1 vs MOD2 ******")
    for j = 1:3
        disp("hole j = " + num2str(j))
        [p,h,stats] = signrank(Errors_mod1(:,j),Errors_mod2(:,j))
    end
    disp("****** ------------------------------ ******")
    disp("")

    disp("****** WILCOXON SIGNED-RANK (paired) on the total (compound) error distribution for MOD1 vs MOD2, reported in Fig. 8C ******")
    [p,h,stats] = signrank(Errors_Sum1,Errors_Sum2)
    disp("****** ------------------------------ ******")
    disp("")

    disp("****** FRIEDMAN (OMNIBUS, across hole sizes) with MOD1 ******")
    [pOmni1,tblOmni1,statsOmni1] = friedman(Errors_mod1)
    disp("Omnibus chi2 = " + num2str(tblOmni1{2,5}) + ", omnibus p = " + num2str(pOmni1))
    disp("--- POST-HOC pairwise (Bonferroni-corrected), MOD1 ---")
    [posthoc1,means1,~,gnames1] = multcompare(statsOmni1,"CriticalValueType","bonferroni");
    for r = 1:size(posthoc1,1)
        disp(gnames1(posthoc1(r,1)) + " vs " + gnames1(posthoc1(r,2)) + ...
            ": p_bonferroni = " + num2str(posthoc1(r,6)))
    end
    disp("****** ------------------------------ ******")
    disp("")

    disp("****** FRIEDMAN (OMNIBUS, across hole sizes) with MOD2 ******")
    [pOmni2,tblOmni2,statsOmni2] = friedman(Errors_mod2)
    disp("Omnibus chi2 = " + num2str(tblOmni2{2,5}) + ", omnibus p = " + num2str(pOmni2))
    disp("--- POST-HOC pairwise (Bonferroni-corrected), MOD2 ---")
    [posthoc2,means2,~,gnames2] = multcompare(statsOmni2,"CriticalValueType","bonferroni");
    for r = 1:size(posthoc2,1)
        disp(gnames2(posthoc2(r,1)) + " vs " + gnames2(posthoc2(r,2)) + ...
            ": p_bonferroni = " + num2str(posthoc2(r,6)))
    end
    disp("****** ------------------------------ ******")
    disp("")

    disp("****** FRIEDMAN (OMNIBUS, across hole sizes), participant-level aggregate across MOD1+MOD2 (N=8 independent blocks), reported in Fig. 8C ******")
    [pOmniSubj,tblOmniSubj,statsOmniSubj] = friedman(Errors_bySubject)
    disp("Omnibus chi2 = " + num2str(tblOmniSubj{2,5}) + ", omnibus p = " + num2str(pOmniSubj))
    disp("--- POST-HOC pairwise (Bonferroni-corrected) ---")
    [posthocSubj,meansSubj,~,gnamesSubj] = multcompare(statsOmniSubj,"CriticalValueType","bonferroni");
    for r = 1:size(posthocSubj,1)
        disp(gnamesSubj(posthocSubj(r,1)) + " vs " + gnamesSubj(posthocSubj(r,2)) + ...
            ": p_bonferroni = " + num2str(posthocSubj(r,6)))
    end
    disp("****** ------------------------------ ******")
    disp("")
end

figure
subplot(121)
hold on
h = dabarplot(Errors,'groups',group_errs);
overlaySubjectScatter(Errors, group_errs, h.gpos);

ylim([0 8])
hold off
subplot(122)
h = dabarplot(Errors_Sum,'groups',group_errs);
overlaySubjectScatter(Errors_Sum, group_errs, h.gpos);

ylim([0 8])

%%

filename = "data_em1_S09.csv";

% custom function to read/import csv data exported in c++ (see at the end of the file)
dataTable = import_CSV_Table(filename);

time = dataTable.time;

time        = time-time(1);
timeMask = time>0.1;
time = time(timeMask);
magPose     = dataTable{timeMask,2:7};
magField    = dataTable{timeMask,17:end};
EE_position = dataTable{timeMask,8:10};
EE_velocity = dataTable{timeMask,11:13};

%%

figure
plot(time,EE_position(:,3))

figure
plot3(EE_position(:,1),EE_position(:,2),EE_position(:,3))
axis equal

%% Auxiliary Function

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