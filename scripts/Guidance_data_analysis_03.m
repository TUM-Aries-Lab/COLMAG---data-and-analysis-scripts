%% FILE SUMMARY
% Purpose: Extract tremor/vibration metrics from ROS bag EE pose during still segments.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; ROS Toolbox; local package dependencies/dabarplot.m.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: ROS bag files in data/prova_stima_tremore.
% Outputs: Segment-wise std-norm metrics, diagnostic plots, and summary tables.
% Run Notes: Manual segment selection is required through interactive clicks (ginput).

warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        bagFile = "data/session_06/2026-03-28-13-40-51.bag";
        addpath("data/session_06")
    end
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % Operations over path
    currentPath = pwd;
    cd ..
    addpath("data/session_06")
    bagFile = "data/session_06/2026-03-28-13-40-51.bag";
    cd(currentPath)
end

%% User settings

addpath("dependencies")
topicName = "/fr3_EE";

manualSegmentSelection = false;

smoothWindowSec = 0.25;    % [s] moving-average window for speed
minManualDuration = 0.0;   % [s] set >0 to ignore very short accidental selections

%% Read position data from bag
bag = rosbagreader(bagFile);
sel = select(bag, "Topic", topicName);
msgs = readMessages(sel, "DataFormat", "struct");

n = numel(msgs);
data = zeros(n, 3);
time = zeros(n, 1);

for i = 1:n
    stamp = msgs{i}.Header.Stamp;
    time(i) = double(stamp.Sec) + double(stamp.Nsec) * 1e-9;
    data(i, 1) = msgs{i}.Pose.Position.X;
    data(i, 2) = msgs{i}.Pose.Position.Y;
    data(i, 3) = msgs{i}.Pose.Position.Z;
end

time = time - time(1);

%% Compute speed and detect almost-still samples
dt = median(diff(time));
if ~isfinite(dt) || dt <= 0
    error("Invalid timestamps: cannot estimate sampling time.");
end
fs = 1 / dt;

dtVec = [NaN; diff(time)];
dPos = [zeros(1, 3); diff(data)];
speed = NaN(n, 1);
validDt = dtVec > 0 & isfinite(dtVec);
speed(validDt) = vecnorm(dPos(validDt, :), 2, 2) ./ dtVec(validDt);
speed = fillmissing(speed, "nearest");
speed = fillmissing(speed, "constant", 0);

smoothWinSamples = max(3, 2 * floor((smoothWindowSec * fs) / 2) + 1); % odd window
speedSmooth = movmean(speed, smoothWinSamples, "omitnan");

if manualSegmentSelection

    %% Manual selection of still segments
    [startIdx, endIdx, durations] = manualSelectSegments( ...
        time, data, speed, speedSmooth, minManualDuration);

    nSeg = numel(startIdx);
    if nSeg == 0
        error("No manual segment selected.");
    end

    %% Compute per-segment vibration metrics (std only)

    stdXYZ = zeros(nSeg, 3);
    stdNorm = zeros(nSeg, 1);

    for s = 1:nSeg
        idx = startIdx(s):endIdx(s);
        seg = data(idx, :);
        stdXYZ(s, :) = std(seg, 0, 1);           % [stdX, stdY, stdZ]
        stdNorm(s) = norm(stdXYZ(s, :), 2);      % ||[stdX stdY stdZ]||
    end

    % Requested array: one value per segment (mean norm-std for each segment)
    meanNormStdArray = stdNorm;
    meanNormStdArray_mm = meanNormStdArray * 1e3;

    segmentStats = table((1:nSeg)', time(startIdx), time(endIdx), durations, ...
        stdXYZ(:, 1), stdXYZ(:, 2), stdXYZ(:, 3), ...
        stdNorm);
    segmentStats.Properties.VariableNames = { ...
        'segment', 'tStart_s', 'tEnd_s', 'duration_s', ...
        'stdX_m', 'stdY_m', 'stdZ_m', 'stdNorm_m'};

    %%

    figure
    h = dabarplot(stdNorm([1:17,19:24,26:31]),'errorbars','SD',...
        'barspacing',0.8);
    scatter(ones(size(stdNorm([1:17,19:24,26:31]))),stdNorm([1:17,19:24,26:31]),"filled","k")

    %% Plot 1: Full signal + manually selected still segments
    fig1 = figure("Color", "w", "Name", "Still Segment Detection");
    tlo1 = tiledlayout(fig1, 4, 1, "TileSpacing", "compact", "Padding", "compact");
    yNames = ["x [m]", "y [m]", "z [m]"];
    segmentColors = turbo(nSeg);

    for c = 1:3
        ax = nexttile(tlo1, c);
        plot(ax, time, data(:, c), "LineWidth", 0.9, "Color", [0.70 0.70 0.70]);
        hold(ax, "on");
        for s = 1:nSeg
            idx = startIdx(s):endIdx(s);
            plot(ax, time(idx), data(idx, c), "LineWidth", 1.7, "Color", segmentColors(s, :));
        end
        grid(ax, "on");
        ylabel(ax, yNames(c));
        if c == 1
            title(ax, "End-effector position with color-coded manually selected segments");
            for s = 1:nSeg
                xMid = 0.5 * (time(startIdx(s)) + time(endIdx(s)));
                text(ax, xMid, max(data(:, c)), sprintf("%d", s), ...
                    "Color", segmentColors(s, :), "FontWeight", "bold", ...
                    "HorizontalAlignment", "center", "VerticalAlignment", "top", ...
                    "Clipping", "on");
            end
        end
    end

    ax4 = nexttile(tlo1, 4);
    hInstant = plot(ax4, time, speed, "Color", [0.55 0.55 0.55], "LineWidth", 0.8);
    hold(ax4, "on");
    hSmooth = plot(ax4, time, speedSmooth, "Color", [0.90 0.33 0.10], "LineWidth", 1.2);
    hSeg = gobjects(nSeg, 1);
    for s = 1:nSeg
        idx = startIdx(s):endIdx(s);
        hSeg(s) = plot(ax4, time(idx), speedSmooth(idx), "LineWidth", 1.7, "Color", segmentColors(s, :));
    end
    grid(ax4, "on");
    xlabel(ax4, "Time [s]");
    ylabel(ax4, "Speed [m/s]");
    legend(ax4, [hInstant, hSmooth, hSeg(1)], "instant", "smoothed", "selected segments", "Location", "best");
    linkaxes(findall(fig1, "Type", "axes"), "x");

    %% Plot 2: Detrended segment traces (vibration inside each segment)
    nPlot = min(nSeg, 8);
    [~, sortByDuration] = sort(durations, "descend");
    plotSegments = sortByDuration(1:nPlot);
    fig2 = figure("Color", "w", "Name", "Still Segment Details");
    tlo2 = tiledlayout(fig2, nPlot, 1, "TileSpacing", "compact", "Padding", "compact");

    for p = 1:nPlot
        s = plotSegments(p);
        idx = startIdx(s):endIdx(s);
        tRel = time(idx) - time(startIdx(s));
        seg = data(idx, :);
        seg = (seg - mean(seg, 1)) * 1e3; % detrended around segment mean, in mm

        ax = nexttile(tlo2, p);
        plot(ax, tRel, seg(:, 1), "LineWidth", 1.0); hold(ax, "on");
        plot(ax, tRel, seg(:, 2), "LineWidth", 1.0);
        plot(ax, tRel, seg(:, 3), "LineWidth", 1.0);
        yline(ax, 0, ":", "HandleVisibility", "off");
        grid(ax, "on");
        ylabel(ax, sprintf("Seg %d [mm]", s));

        if p == 1
            title(ax, "Detrended position during still segments");
            legend(ax, "x", "y", "z", "Location", "eastoutside");
        end
        if p == nPlot
            xlabel(ax, "Time in segment [s]");
        end
    end


else

    load data/session_06/segmentsStats.mat
    load data/session_06/stdNormsData.mat

    meanNormStdArray = stdNorm;
    meanNormStdArray_mm = meanNormStdArray * 1e3;
    nSeg = length(stdNorm);

end

%% Plot 3: Bar plot of norm(std) for each segment
fig3 = figure("Color", "w", "Name", "Norm STD per Segment");
ax = axes(fig3);
b = bar(ax, 1:nSeg, meanNormStdArray_mm, 0.75, "FaceColor", "flat", "EdgeColor", "none");
b.CData = turbo(nSeg);
grid(ax, "on");
xlabel(ax, "Still segment");
ylabel(ax, "norm(std(x,y,z)) [mm]");
title(ax, "Segment vibration amplitude from std norm");
xticks(ax, 1:nSeg);

try
    if(test)
    end
catch exception
    %% Console summary
    fprintf("\nManually selected %d still segments.\n", nSeg);
    disp(segmentStats);
end


% ------------------------- Local function ------------------------------
function [startIdx, endIdx, durations] = manualSelectSegments(time, data, speed, speedSmooth, minManualDuration)
figSel = figure("Color", "w", "Name", "Manual Segment Selection");
tlo = tiledlayout(figSel, 4, 1, "TileSpacing", "compact", "Padding", "compact");

ax1 = nexttile(tlo, 1);
plot(ax1, time, data(:, 1), "Color", [0.05 0.35 0.75], "LineWidth", 1.0);
grid(ax1, "on");
ylabel(ax1, "x [m]");
title(ax1, "Click START/END pairs of still segments. Press Enter when done.");

ax2 = nexttile(tlo, 2);
plot(ax2, time, data(:, 2), "Color", [0.05 0.35 0.75], "LineWidth", 1.0);
grid(ax2, "on");
ylabel(ax2, "y [m]");

ax3 = nexttile(tlo, 3);
plot(ax3, time, data(:, 3), "Color", [0.05 0.35 0.75], "LineWidth", 1.0);
grid(ax3, "on");
ylabel(ax3, "z [m]");

ax4 = nexttile(tlo, 4);
plot(ax4, time, speed, "Color", [0.6 0.6 0.6], "LineWidth", 0.8); hold(ax4, "on");
plot(ax4, time, speedSmooth, "Color", [0.90 0.33 0.10], "LineWidth", 1.2);
grid(ax4, "on");
xlabel(ax4, "Time [s]");
ylabel(ax4, "Speed [m/s]");
legend(ax4, "instant", "smoothed", "Location", "best");

linkaxes([ax1, ax2, ax3, ax4], "x");

fprintf("\nManual selection:\n");
fprintf(" - Click start/end boundaries in time (pairs of clicks).\n");
fprintf(" - Example: click [start1, end1, start2, end2, ...]\n");
fprintf(" - Press Enter when finished.\n\n");

figure(figSel);
[xClick, ~] = ginput;

if isempty(xClick)
    startIdx = [];
    endIdx = [];
    durations = [];
    return
end

xClick = xClick(:);
if mod(numel(xClick), 2) == 1
    warning("Odd number of clicks: last click ignored.");
    xClick(end) = [];
end

if isempty(xClick)
    startIdx = [];
    endIdx = [];
    durations = [];
    return
end

startTimes = xClick(1:2:end);
endTimes = xClick(2:2:end);

swapMask = endTimes < startTimes;
tmp = startTimes(swapMask);
startTimes(swapMask) = endTimes(swapMask);
endTimes(swapMask) = tmp;

startTimes = max(startTimes, time(1));
endTimes = min(endTimes, time(end));

startIdx = round(interp1(time, 1:numel(time), startTimes, "nearest", "extrap"));
endIdx = round(interp1(time, 1:numel(time), endTimes, "nearest", "extrap"));

startIdx = max(1, min(numel(time), startIdx(:)));
endIdx = max(1, min(numel(time), endIdx(:)));

pairs = [startIdx, endIdx];
pairs = pairs(pairs(:, 2) >= pairs(:, 1), :);
if isempty(pairs)
    startIdx = [];
    endIdx = [];
    durations = [];
    return
end

pairs = sortrows(pairs, 1);
merged = pairs(1, :);
for i = 2:size(pairs, 1)
    if pairs(i, 1) <= merged(end, 2) + 1
        merged(end, 2) = max(merged(end, 2), pairs(i, 2));
    else
        merged = [merged; pairs(i, :)]; %#ok<AGROW>
    end
end

startIdx = merged(:, 1);
endIdx = merged(:, 2);
durations = time(endIdx) - time(startIdx);

if minManualDuration > 0
    keep = durations >= minManualDuration;
    startIdx = startIdx(keep);
    endIdx = endIdx(keep);
    durations = durations(keep);
end
end