%% FILE SUMMARY
% Purpose: Build an LSTM pipeline to estimate magnetic fields from robot link orientations.
% Last Updated: 2026-04-14.
% Dependencies: MATLAB; ROS Toolbox; System Identification Toolbox; Deep Learning Toolbox required for trainNetwork/lstmLayer blocks.
% MATLAB Version: Tested on MATLAB R2025b; scripts using Name=Value syntax are recommended for R2021a or newer.
% Data: ROS bag files in data/prova_stima_rumore_robot.
% Outputs: Trained networks, prediction-vs-ground-truth plots, and norm-error histograms.
% Run Notes: Long-running training script; requires Deep Learning Toolbox.


warning off
try
    if(test)
        disp("TEST MODE ON "+scriptName)
        addpath("data/session_05")
        bag = rosbagreader("2026-03-24-13-57-45.bag");
    end
catch exception
    clc
    clear
    close all
    disp("TEST MODE OFF")
    % operations over path
    currentPath = pwd;
    cd ..
    addpath("data/session_05")
    bag = rosbagreader("2026-03-24-13-57-45.bag");
    cd(currentPath)
end

addpath("dependencies")
TRAIN_NN = false;

% bag.AvailableTopics

sel = select(bag, "Topic", "/L3/mag_field_data");
msgs = readMessages(sel, "DataFormat", "struct");
numSens = 8;

for i = 1:size(msgs,1)
    magdata = reshape(msgs{i,1}.Data,3,numSens)';
    for j = 1:numSens
        data_1(i,:,j) = magdata(j,:);
    end
end

sel = select(bag, "Topic", "/L5/mag_field_data");
msgs = readMessages(sel, "DataFormat", "struct");
numSens = 8;

for i = 1:size(msgs,1)
    magdata = reshape(msgs{i,1}.Data,3,numSens)';
    for j = 1:numSens
        data_2(i,:,j) = magdata(j,:);
    end
end

%%

quat_struct = struct();
rotm_struct = struct();
availableTopicNames = string(bag.AvailableTopics.Properties.RowNames);

for linkIdx = 1:7
    fieldName = "fr3_link" + linkIdx + "_pose";
    topicName = "/" + fieldName;

    if ~any(availableTopicNames == topicName)
        warning("Topic %s not available in bag. Storing empty matrix.", topicName);
        quat_struct.(fieldName) = zeros(0,4);
        rotm_struct.(fieldName) = zeros(3,3,0);
        continue
    end

    sel = select(bag, "Topic", topicName);
    msgs = readMessages(sel, "DataFormat", "struct");

    N = numel(msgs);
    quatMat = zeros(N,4);      % [qw qx qy qz]
    rotmMat = zeros(3,3,N);

    for i = 1:N
        qx = msgs{i, 1}.Pose.Orientation.X;
        qy = msgs{i, 1}.Pose.Orientation.Y;
        qz = msgs{i, 1}.Pose.Orientation.Z;
        qw = msgs{i, 1}.Pose.Orientation.W;

        quatMat(i,:) = [qw qx qy qz];
        rotmMat(:,:,i) = quat2rotm(quatMat(i,:));
    end

    quat_struct.(fieldName) = quatMat;
    rotm_struct.(fieldName) = rotmMat;
end

rotm1 = rotm_struct.fr3_link4_pose;
rotm2 = rotm_struct.fr3_link2_pose;

if (TRAIN_NN)

    % -------------------------------------------------------------------------
    % Train NN: input = flattened orientation of all 7 links (9*7 = 63 nodes),
    % output = magnetic field of 8 sensors (3*8 = 24 nodes).
    % Time information is modeled with an LSTM sequence-to-sequence regressor.
    % -------------------------------------------------------------------------

    linkFields = compose("fr3_link%d_pose", 1:7);
    nLinks = numel(linkFields);

    rotLen = zeros(1,nLinks);
    for k = 1:nLinks
        fieldName = char(linkFields(k));
        if ~isfield(rotm_struct, fieldName) || isempty(rotm_struct.(fieldName))
            error("Missing or empty rotation data for %s.", fieldName);
        end
        rotLen(k) = size(rotm_struct.(fieldName),3);
    end

    Ncommon = min([rotLen, size(data_1,1), size(data_2,1)]);
    if Ncommon < 50
        warning("Only %d synchronized samples available. Training quality may be limited.", Ncommon);
    end

    % Build input matrix X (N x 63)
    X = zeros(Ncommon, 9*nLinks);
    for i = 1:Ncommon
        colIdx = 1;
        for k = 1:nLinks
            fieldName = char(linkFields(k));
            Rall = rotm_struct.(fieldName);
            R = Rall(:,:,i);
            X(i, colIdx:colIdx+8) = reshape(R,1,9);   % flatten 3x3 -> 1x9
            colIdx = colIdx + 9;
        end
    end

    % Build output matrices Y1 and Y2 (N x 24)
    Y1 = zeros(Ncommon, 3*numSens);
    Y2 = zeros(Ncommon, 3*numSens);
    for i = 1:Ncommon
        Y1(i,:) = reshape(data_1(i,:,:),1,[]);
        Y2(i,:) = reshape(data_2(i,:,:),1,[]);
    end

    % Normalize features/targets
    muX = mean(X,1);
    sigX = std(X,0,1);
    sigX(sigX < 1e-12) = 1;
    Xn = (X - muX) ./ sigX;

    muY1 = mean(Y1,1);
    sigY1 = std(Y1,0,1);
    sigY1(sigY1 < 1e-12) = 1;
    Y1n = (Y1 - muY1) ./ sigY1;

    muY2 = mean(Y2,1);
    sigY2 = std(Y2,0,1);
    sigY2(sigY2 < 1e-12) = 1;
    Y2n = (Y2 - muY2) ./ sigY2;

    % Time-aware split (chronological)
    Ntrain = floor(0.70*Ncommon);
    Nval = floor(0.15*Ncommon);
    Ntest = Ncommon - Ntrain - Nval;
    if Ntrain < 10 || Nval < 5 || Ntest < 5
        error("Not enough samples for train/val/test split. Increase bag duration.");
    end

    idxTrain = 1:Ntrain;
    idxVal = (Ntrain+1):(Ntrain+Nval);
    idxTest = (Ntrain+Nval+1):Ncommon;

    % Convert to sequence format: features x time
    XTrain = {Xn(idxTrain,:)'};
    XVal = {Xn(idxVal,:)'};
    XTest = {Xn(idxTest,:)'};

    Y1Train = {Y1n(idxTrain,:)'};
    Y1Val = {Y1n(idxVal,:)'};

    Y2Train = {Y2n(idxTrain,:)'};
    Y2Val = {Y2n(idxVal,:)'};

    layers = [
        sequenceInputLayer(9*7, "Name","input")
        lstmLayer(128, "OutputMode","sequence", "Name","lstm")
        fullyConnectedLayer(64, "Name","fc1")
        reluLayer("Name","relu1")
        fullyConnectedLayer(3*8, "Name","fc_out")
        regressionLayer("Name","reg")
        ];

    opts1 = trainingOptions("adam", ...
        "MaxEpochs", 250, ...
        "MiniBatchSize", 1, ...
        "InitialLearnRate", 1e-3, ...
        "GradientThreshold", 1, ...
        "Shuffle", "never", ...       % keep temporal order
        "ValidationData", {XVal, Y1Val}, ...
        "ValidationFrequency", 20, ...
        "Verbose", true, ...
        "Plots", "training-progress");

    % Train model for data_1
    net_data1 = trainNetwork(XTrain, Y1Train, layers, opts1);

    % Train model for data_2
    opts2 = trainingOptions("adam", ...
        "MaxEpochs", 250, ...
        "MiniBatchSize", 1, ...
        "InitialLearnRate", 1e-3, ...
        "GradientThreshold", 1, ...
        "Shuffle", "never", ...       % keep temporal order
        "ValidationData", {XVal, Y2Val}, ...
        "ValidationFrequency", 20, ...
        "Verbose", true, ...
        "Plots", "training-progress");
    net_data2 = trainNetwork(XTrain, Y2Train, layers, opts2);

    % Evaluate on test set
    Y1PredN = predict(net_data1, XTest, "MiniBatchSize", 1);
    Y2PredN = predict(net_data2, XTest, "MiniBatchSize", 1);

    Y1Pred = Y1PredN{1}' .* sigY1 + muY1;
    Y2Pred = Y2PredN{1}' .* sigY2 + muY2;

    Y1True = Y1(idxTest,:);
    Y2True = Y2(idxTest,:);

    rmse_data1 = sqrt(mean((Y1Pred - Y1True).^2, "all"));
    rmse_data2 = sqrt(mean((Y2Pred - Y2True).^2, "all"));

    fprintf("Test RMSE data_1: %.4f\n", rmse_data1);
    fprintf("Test RMSE data_2: %.4f\n", rmse_data2);

    %%
    % Plot estimation vs ground truth on test set
    tTest = idxTest(:);
    fieldLabels = ["Bx","By","Bz"];

    for s = 1:numSens
        cols = (3*(s-1)+1):(3*s);

        figure("Name", sprintf("data_1 Sensor %d: Estimated vs Ground Truth", s), ...
            "Color", "w");
        tiledlayout(3,1);
        for c = 1:3
            nexttile;
            plot(tTest, Y1True(:,cols(c)), "k-", "LineWidth", 1.0); hold on;
            plot(tTest, Y1Pred(:,cols(c)), "r--", "LineWidth", 1.0);
            grid on;
            ylabel(fieldLabels(c));
            if c == 1
                title(sprintf("data\\_1 - Sensor %d", s));
            end
            if c == 3
                xlabel("Time index");
            end
            legend("Ground truth","Estimated","Location","best");
        end

        figure("Name", sprintf("data_2 Sensor %d: Estimated vs Ground Truth", s), ...
            "Color", "w");
        tiledlayout(3,1);
        for c = 1:3
            nexttile;
            plot(tTest, Y2True(:,cols(c)), "k-", "LineWidth", 1.0); hold on;
            plot(tTest, Y2Pred(:,cols(c)), "r--", "LineWidth", 1.0);
            grid on;
            ylabel(fieldLabels(c));
            if c == 1
                title(sprintf("data\\_2 - Sensor %d", s));
            end
            if c == 3
                xlabel("Time index");
            end
            legend("Ground truth","Estimated","Location","best");
        end
    end

    % Compute norms over time for each sensor:
    % 1) Ground truth magnetic field norm ||[Bx By Bz]||
    % 2) Estimation error norm ||[Bx By Bz]_true - [Bx By Bz]_est||
    gt_norm_data1 = zeros(size(Y1True,1), numSens);
    err_norm_data1 = zeros(size(Y1True,1), numSens);
    gt_norm_data2 = zeros(size(Y2True,1), numSens);
    err_norm_data2 = zeros(size(Y2True,1), numSens);

    for s = 1:numSens
        cols = (3*(s-1)+1):(3*s);

        gtVec1 = Y1True(:,cols);
        predVec1 = Y1Pred(:,cols);
        gt_norm_data1(:,s) = sqrt(sum(gtVec1.^2,2));
        err_norm_data1(:,s) = sqrt(sum((gtVec1 - predVec1).^2,2));

        gtVec2 = Y2True(:,cols);
        predVec2 = Y2Pred(:,cols);
        gt_norm_data2(:,s) = sqrt(sum(gtVec2.^2,2));
        err_norm_data2(:,s) = sqrt(sum((gtVec2 - predVec2).^2,2));
    end

    % Flatten (all sensors, all test times) for histogram comparison
    gt_norm_data1_all = gt_norm_data1(:);
    err_norm_data1_all = err_norm_data1(:);
    gt_norm_data2_all = gt_norm_data2(:);
    err_norm_data2_all = err_norm_data2(:);

    % Store everything useful in a struct
    nn_models = struct();
    nn_models.net_data1 = net_data1;
    nn_models.net_data2 = net_data2;
    nn_models.inputNormalization.mu = muX;
    nn_models.inputNormalization.sigma = sigX;
    nn_models.output1Normalization.mu = muY1;
    nn_models.output1Normalization.sigma = sigY1;
    nn_models.output2Normalization.mu = muY2;
    nn_models.output2Normalization.sigma = sigY2;
    nn_models.rmse_data1 = rmse_data1;
    nn_models.rmse_data2 = rmse_data2;
    nn_models.linkFields = linkFields;
    nn_models.numInputNodes = 9*7;
    nn_models.numOutputNodes = 3*8;
    nn_models.sequenceLengthUsed = Ncommon;
    nn_models.norms.data_1.groundTruth = gt_norm_data1;
    nn_models.norms.data_1.error = err_norm_data1;
    nn_models.norms.data_2.groundTruth = gt_norm_data2;
    nn_models.norms.data_2.error = err_norm_data2;
    nn_models.norms.data_1.groundTruthAll = gt_norm_data1_all;
    nn_models.norms.data_1.errorAll = err_norm_data1_all;
    nn_models.norms.data_2.groundTruthAll = gt_norm_data2_all;
    nn_models.norms.data_2.errorAll = err_norm_data2_all;

else

    load("data\session_05\NNResults01.mat")

    net_data1 = nn_models.net_data1;
    net_data2 = nn_models.net_data2;
    muX = nn_models.inputNormalization.mu;
    sigX = nn_models.inputNormalization.sigma;
    muY1 = nn_models.output1Normalization.mu;
    sigY1 = nn_models.output1Normalization.sigma;
    muY2 = nn_models.output2Normalization.mu;
    sigY2 = nn_models.output2Normalization.sigma;
    rmse_data1 = nn_models.rmse_data1;
    rmse_data2 = nn_models.rmse_data2;
    linkFields = nn_models.linkFields;
    Ncommon = nn_models.sequenceLengthUsed;
    gt_norm_data1 = nn_models.norms.data_1.groundTruth;
    err_norm_data1 = nn_models.norms.data_1.error;
    gt_norm_data2 = nn_models.norms.data_2.groundTruth;
    err_norm_data2 = nn_models.norms.data_2.error;
    gt_norm_data1_all = nn_models.norms.data_1.groundTruthAll;
    err_norm_data1_all = nn_models.norms.data_1.errorAll;
    gt_norm_data2_all = nn_models.norms.data_2.groundTruthAll;
    err_norm_data2_all = nn_models.norms.data_2.errorAll;

end



%% Display picture of the dynamic calibration for peripheral detectors

figure("Name","Histogram of Norms: Ground Truth vs Estimation Error", "Color","w");
tiledlayout(2,1);

nexttile;
histogram(gt_norm_data1_all, "Normalization","probability", "FaceAlpha",0.55); hold on;
histogram(err_norm_data1_all, "Normalization","probability", "FaceAlpha",0.55);
grid on;
title("data\_1");
xlabel("Norm value");
ylabel("Probability");
legend("||B|| ground truth", "||B_{gt} - B_{est}||", "Location","best");

nexttile;
histogram(gt_norm_data2_all, "Normalization","probability", "FaceAlpha",0.55); hold on;
histogram(err_norm_data2_all, "Normalization","probability", "FaceAlpha",0.55);
grid on;
title("data\_2");
xlabel("Norm value");
ylabel("Probability");
legend("||B|| ground truth", "||B_{gt} - B_{est}||", "Location","best");

% Histogram per individual sensor
for s = 1:numSens
    figure("Name", sprintf("Sensor %d Norm Histograms", s), "Color","w");
    tiledlayout(2,1);

    nexttile;
    histogram(gt_norm_data1(:,s), "Normalization","probability", "FaceAlpha",0.55); hold on;
    histogram(err_norm_data1(:,s), "Normalization","probability", "FaceAlpha",0.55);
    grid on;
    title(sprintf("data\\_1 - Sensor %d", s));
    xlabel("Norm value");
    ylabel("Probability");
    legend("||B|| ground truth", "||B_{gt} - B_{est}||", "Location","best");

    nexttile;
    histogram(gt_norm_data2(:,s), "Normalization","probability", "FaceAlpha",0.55); hold on;
    histogram(err_norm_data2(:,s), "Normalization","probability", "FaceAlpha",0.55);
    grid on;
    title(sprintf("data\\_2 - Sensor %d", s));
    xlabel("Norm value");
    ylabel("Probability");
    legend("||B|| ground truth", "||B_{gt} - B_{est}||", "Location","best");
end