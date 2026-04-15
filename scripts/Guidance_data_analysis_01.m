%% FILE SUMMARY
% Purpose: Analyze guidance user-study results (errors, NASA-TLX, trajectory metrics).
% Last Updated: 2026-04-14.
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
end

addpath("dependencies")

%% Data Collected via questionaires and monitoring results in pick and place

Errors_mod1 = [ 0	0	0    % S1
    0	0	0    % S2
    0	0	2    % S3
    0	3	4    % S4
    0	0	1    % S5
    4	1	3    % S6
    0	1	2    % S7
    1	1	4    % S8
    0	4	2 ]; % S9

Errors_mod2 = [ 1   1	4    % S1
    1	0	2    % S2
    0	1	1    % S3
    1	3	3    % S4
    1	0	5    % S5
    0	1	4    % S6
    0	4	3    % S7
    1	2	3    % S8
    0	0	5 ]; % S9

NASA_mod1 = [   4 5 1 2 3 2     % S1
    16 3 3 2 3 1    % S2
    7 3 1 3 13 2    % S3
    5 2 2 2 2 1     % S4
    4 2 4 2 6 1     % S5
    9 6 10 4 4 6    % S6
    5 1 1 12 7 2    % S7
    7 2 2 6 10 4    % S8
    4 4 1 9 4 1];   % S9

NASA_mod2 = [   7 8 2 2 2 2     % S1
    15 2 3 17 4 6   % S2
    2 2 2 3 1 1     % S3
    6 4 4 3 2 2     % S4
    12 13 10 9 10 8 % S5
    6 2 4 6 2 3     % S6
    3 1 1 6 2 2     % S7
    4 2 5 2 2 2     % S8
    5 5 2 3 5 3];   % S9

%%

NASA = [NASA_mod1; NASA_mod2];
NASA_score = [sum(NASA_mod1,2); sum(NASA_mod2,2)];
group_inx = [ones(1,9) 2*ones(1,9)]';


try
    if(test)
        for j = 1:6
            [p,h,stats] = ranksum(NASA_mod1(:,j),NASA_mod2(:,j));
        end

        figure
        subplot(121)
        hold on
        h = dabarplot(NASA,'groups',group_inx,'errorbars','SE',...
            'barspacing',0.8);

        hold off
        subplot(122)
        hold on
        h2 = dabarplot(NASA_score,'groups',group_inx,'errorbars','SE',...
            'barspacing',0.8);


        [p,h,stats] = ranksum(NASA_score(1:9),NASA_score(10:end));

        %%

        Errors = [Errors_mod1; Errors_mod2];
        group_errs = [ones(1,9) 2*ones(1,9)]';
        
        for j = 1:3
            [p,h,stats] = ranksum(Errors_mod1(:,j),Errors_mod2(:,j));
        end

        Errors_Sum = [sum(Errors_mod1,2); sum(Errors_mod2,2)];

        [p,h,stats] = ranksum(Errors_Sum(1:9),Errors_Sum(10:end));

        [p,h,stats] = friedman(Errors_mod1);
        [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni");

        [p,h,stats] = friedman(Errors_mod2);
        [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni");

    end
catch exception
    for j = 1:6
        [p,h,stats] = ranksum(NASA_mod1(:,j),NASA_mod2(:,j))
    end

    figure
    subplot(121)
    hold on
    h = dabarplot(NASA,'groups',group_inx,'errorbars','SE',...
        'barspacing',0.8);

    hold off
    subplot(122)
    hold on
    h2 = dabarplot(NASA_score,'groups',group_inx,'errorbars','SE',...
        'barspacing',0.8);


    [p,h,stats] = ranksum(NASA_score(1:9),NASA_score(10:end))

    %%

    Errors = [Errors_mod1; Errors_mod2];
    group_errs = [ones(1,9) 2*ones(1,9)]';

    disp("****** WILCOXON on the individual error distribution for MOD ******")
    for j = 1:3
        disp("j = " + num2str(j))
        [p,h,stats] = ranksum(Errors_mod1(:,j),Errors_mod2(:,j))
    end
    disp("****** ------------------------------ ******")
    disp("")

    Errors_Sum = [sum(Errors_mod1,2); sum(Errors_mod2,2)];

    disp("****** WILCOXON on the compound error distribution for MOD ******")
    [p,h,stats] = ranksum(Errors_Sum(1:9),Errors_Sum(10:end))
    disp("****** ------------------------------ ******")
    disp("")

    disp("****** FRIEDMAN on peg size with MOD1 ******")
    [p,h,stats] = friedman(Errors_mod1)
    [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni")
    disp("****** ------------------------------ ******")
    disp("")

    disp("****** FRIEDMAN on peg size with MOD2 ******")
    [p,h,stats] = friedman(Errors_mod2)
    [results,means,~,gnames] = multcompare(stats,"CriticalValueType","bonferroni")
    disp("****** ------------------------------ ******")
    disp("")
end

figure
subplot(121)
hold on
h = daviolinplot(Errors,'groups',group_errs);
ylim([0 6])
hold off
subplot(122)
h = daviolinplot(Errors_Sum,'groups',group_errs);
ylim([0 6])

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