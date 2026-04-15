%% FILE SUMMARY
% Purpose: Enhanced dynamic calibration comparison with compact summary plots per sensor.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; local package dependencies/dabarplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: CSV logs in data/session_fede_2_novembre.
% Outputs: Probability histograms and dabarplot summaries for calibrated/non-calibrated fields.
% Run Notes: Use this version for cleaner publication-ready calibration panel generation.

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath(genpath("data/session_02"))
    end 
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % operations over path
    currentPath = pwd;
    cd ..
    addpath(genpath("data/session_02"))
    cd(currentPath)
end

addpath("dependencies")

%%

filename = "DynamicCalibData.csv";
data = importfile(filename);

B1 = data(:,17:19);

nSamples = size(B1,1);
validDataMask = false(nSamples,1);

for i = 2:nSamples
    if ~isequal(B1(i,:),B1(i-1,:))
        validDataMask(i) = true;
    end
end

B1 = data(validDataMask,17:19);
B2 = data(validDataMask,20:22);
B3 = data(validDataMask,23:25);
B4 = data(validDataMask,26:28);

B1n = data(validDataMask,29:31);
B2n = data(validDataMask,32:34);
B3n = data(validDataMask,35:37);
B4n = data(validDataMask,38:40);

%%

figure
subplot(3,2,1)
histogram(B1(:,1),'Normalization','probability','BinWidth',2)
xlim([-1 1]*40)
ylim([0 0.3])

subplot(3,2,2)
histogram(B1n(:,1),'Normalization','probability','BinWidth',2)
xlim([-1 1]*40)
ylim([0 0.3])

subplot(3,2,3)
histogram(B1(:,2),'Normalization','probability','BinWidth',2)
xlim([-1 1]*40)
ylim([0 0.3])

subplot(3,2,4)
histogram(B1n(:,2),'Normalization','probability','BinWidth',2)
xlim([-1 1]*40)
ylim([0 0.3])

subplot(3,2,5)
histogram(B1(:,3),'Normalization','probability','BinWidth',2)
xlim([-1 1]*40)
ylim([0 0.3])

subplot(3,2,6)
histogram(B1n(:,3),'Normalization','probability','BinWidth',2)
xlim([-1 1]*40)
ylim([0 0.3])

%%

BINSIZE = 1.7;

[B1_f,B1_xf] = kde(B1(:,1),Bandwidth=1);
[B1n_f,B1n_xf] = kde(B1n(:,1),Bandwidth=1);

figure
subplot(3,1,1)
hold on
histogram(B1(:,1),'Normalization','probability','BinWidth',BINSIZE)
histogram(B1n(:,1),'Normalization','probability','BinWidth',BINSIZE)
plot(B1_xf,B1_f,'b')
plot(B1n_xf,B1n_f,'r')
hold off
xlim([-1 1]*40)
ylim([0 0.4])

[B1_f,B1_xf] = kde(B1(:,2),Bandwidth=2);
[B1n_f,B1n_xf] = kde(B1n(:,2),Bandwidth=2);

subplot(3,1,2)
hold on
histogram(B1(:,2),'Normalization','probability','BinWidth',BINSIZE)
histogram(B1n(:,2),'Normalization','probability','BinWidth',BINSIZE)
plot(B1_xf,B1_f,'b')
plot(B1n_xf,B1n_f,'r')
hold off
xlim([-1 1]*40)
ylim([0 0.4])


[B1_f,B1_xf] = kde(B1(:,3),Bandwidth=1);
[B1n_f,B1n_xf] = kde(B1n(:,3),Bandwidth=1);

subplot(3,1,3)
hold on
histogram(B1(:,3),'Normalization','probability','BinWidth',BINSIZE)
histogram(B1n(:,3),'Normalization','probability','BinWidth',BINSIZE)
plot(B1_xf,B1_f,'b')
plot(B1n_xf,B1n_f,'r')
hold off
xlim([-1 1]*40)
ylim([0 0.4])

%%

figure(Units="normalized",Position=[0.1 0.3 0.8 0.3])

BINWIDTH = 0.5;

subplot(2,4,1)
hold on
histogram(vecnorm(B1,2,2),'Normalization','probability','BinWidth',BINWIDTH)
histogram(vecnorm(B1n,2,2),'Normalization','probability','BinWidth',BINWIDTH)
xline(median(vecnorm(B1n,2,2)),'k--')
txt = num2str(round(median(vecnorm(B1n,2,2))*100)/100) + " " + "\rightarrow";
text(median(vecnorm(B1n,2,2)),0.08,txt,'HorizontalAlignment','right')
xline(median(vecnorm(B1,2,2)),'k--')
txt = "\leftarrow" + " " + num2str(round(median(vecnorm(B1,2,2))*100)/100) ;
text(median(vecnorm(B1,2,2)),0.06,txt,'HorizontalAlignment','left')
xlabel("Magnetic Field Norm (\muT)")
axis([0 50 0 0.12])
hold off

subplot(2,4,5)
hold on
dabarplot([vecnorm(B1,2,2) vecnorm(B1n,2,2)],'errorbars','SD',...
    'barspacing',0.8)
ylim([0 50])
camroll(-90)

subplot(2,4,2)
hold on
histogram(vecnorm(B2,2,2),'Normalization','probability','BinWidth',BINWIDTH)
histogram(vecnorm(B2n,2,2),'Normalization','probability','BinWidth',BINWIDTH)
xline(median(vecnorm(B2n,2,2)),'k--')
txt = num2str(round(median(vecnorm(B2n,2,2))*100)/100) + " " + "\rightarrow";
text(median(vecnorm(B2n,2,2)),0.08,txt,'HorizontalAlignment','right')
xline(median(vecnorm(B2,2,2)),'k--')
txt = "\leftarrow" + " " + num2str(round(median(vecnorm(B2,2,2))*100)/100) ;
text(median(vecnorm(B2,2,2)),0.06,txt,'HorizontalAlignment','left')
xlabel("Magnetic Field Norm (\muT)")
axis([0 50 0 0.12])
hold off

subplot(2,4,6)
hold on
dabarplot([vecnorm(B2,2,2) vecnorm(B2n,2,2)],'errorbars','SD',...
    'barspacing',0.8)
ylim([0 50])
camroll(-90)

subplot(2,4,3)
hold on
histogram(vecnorm(B3,2,2),'Normalization','probability','BinWidth',BINWIDTH)
histogram(vecnorm(B3n,2,2),'Normalization','probability','BinWidth',BINWIDTH)
xlabel("Magnetic Field Norm (\muT)")
xline(median(vecnorm(B3n,2,2)),'k--')
txt = num2str(round(median(vecnorm(B3n,2,2))*100)/100) + " " + "\rightarrow";
text(median(vecnorm(B3n,2,2)),0.08,txt,'HorizontalAlignment','right')
xline(median(vecnorm(B3,2,2)),'k--')
txt = "\leftarrow" + " " + num2str(round(median(vecnorm(B3,2,2))*100)/100) ;
text(median(vecnorm(B3,2,2)),0.06,txt,'HorizontalAlignment','left')
axis([0 50 0 0.12])
hold off

subplot(2,4,7)
hold on
dabarplot([vecnorm(B3,2,2) vecnorm(B3n,2,2)],'errorbars','SD',...
    'barspacing',0.8)
ylim([0 50])
camroll(-90)

subplot(2,4,4)
hold on
histogram(vecnorm(B4,2,2),'Normalization','probability','BinWidth',BINWIDTH)
histogram(vecnorm(B4n,2,2),'Normalization','probability','BinWidth',BINWIDTH)
xlabel("Magnetic Field Norm (\muT)")
xline(median(vecnorm(B4n,2,2)),'k--')
txt = num2str(round(median(vecnorm(B4n,2,2))*100)/100) + " " + "\rightarrow";
text(median(vecnorm(B4n,2,2)),0.08,txt,'HorizontalAlignment','right')
xline(median(vecnorm(B4,2,2)),'k--')
txt = "\leftarrow" + " " + num2str(round(median(vecnorm(B4,2,2))*100)/100) ;
text(median(vecnorm(B4,2,2)),0.06,txt,'HorizontalAlignment','left')
axis([0 50 0 0.12])
hold off

subplot(2,4,8)
hold on
dabarplot([vecnorm(B4,2,2) vecnorm(B4n,2,2)],'errorbars','SD',...
    'barspacing',0.8)
ylim([0 50])
camroll(-90)

%%

function data = importfile(filename, dataLines)
% Auto-generated by MATLAB on 03-Nov-2024 01:22:30

% Input handling

% If dataLines is not specified, define defaults
if nargin < 2
    dataLines = [2, Inf];
end

% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 42);

% Specify range and delimiter
opts.DataLines = dataLines;
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["time", "mag_pos_x", "mag_pos_y", "mag_pos_z", "mag_orien_x", "mag_orien_y", "mag_orien_z", "ee_x", "ee_y", "ee_z", "v_ee_x", "v_ee_y", "v_ee_z", "F_ee_x", "F_ee_y", "F_ee_z", "s1_x", "s1_y", "s1_z", "s2_x", "s2_y", "s2_z", "s3_x", "s3_y", "s3_z", "s4_x", "s4_y", "s4_z", "n_s1_x", "n_s1_y", "n_s1_z", "n_s2_x", "n_s2_y", "n_s2_z", "n_s3_x", "n_s3_y", "n_s3_z", "n_s4_x", "n_s4_y", "n_s4_z", "ls_compute_time", "r2"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Import the data
data = readtable(filename, opts);

% Convert to output type
data = table2array(data);
end