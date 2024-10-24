% mainSLAMLoop.m
% Main script to run the SLAM system using Basler Flexx2 in MATLAB

clc; clear; close all;

% Initialize Camera
camera = initializeCamera();

% Perform Camera Calibration (if not already calibrated)
if ~exist('cameraCalib.mat', 'file')
    disp('Camera calibration not found. Starting calibration...');
    calibrateCameraManual(); % User performs calibration via GUI
    if ~exist('cameraCalib.mat', 'file')
        error('Calibration not completed. "cameraCalib.mat" not found.');
    end
end

% Load Camera Intrinsics
load('cameraCalib.mat'); % Ensure cameraCalib.mat contains 'cameraParams'
if ~exist('cameraParams', 'var') || ~isfield(cameraParams, 'Intrinsics')
    error('Invalid calibration data. "cameraCalib.mat" must contain "cameraParams.Intrinsics".');
end
intrinsics = cameraParams.Intrinsics;
disp('Loaded camera intrinsics from cameraCalib.mat.');

% Initialize Global Map and Pose Log
globalMap = [];
poseLog = [];

% Initialize Previous Frame Variables
prevGrayImage = [];
prevPointCloud = [];

% Initialize Figures for Visualization
figureMap = figure('Name', 'Global Map', 'NumberTitle', 'off');
mapHandle = scatter3(0, 0, 0, 1, 'b.');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Global Map');
grid on;
hold on;

figureTraj = figure('Name', 'Trajectory', 'NumberTitle', 'off');
trajHandle = plot3(0, 0, 0, 'r-', 'LineWidth', 2);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Astronaut Trajectory');
grid on;
hold on;

% Main SLAM Loop
disp('Starting SLAM...');
try
    while ishandle(figureMap) && ishandle(figureTraj)
        % Capture Data
        data = camera.getData();
        grayValue = []; x = []; y = []; z = [];

        % Assign Grayscale and Depth Data
        if isfield(data, 'grayValue') && isfield(data, 'x') && isfield(data, 'y') && isfield(data, 'z')
            grayValue = data.grayValue;
            x = data.x;
            y = data.y;
            z = data.z;
        else
            warning('Necessary fields (grayValue, x, y, z) not found in data structure.');
            continue; % Skip to next iteration
        end

        % Preprocess Data
        [filteredDepth, enhancedGray, currentPointCloud] = preprocessData(grayValue, x, y, z, intrinsics, 0.1);

        % Check if pointCloud is not empty
        if isempty(currentPointCloud)
            warning('Empty point cloud detected. Skipping frame.');
            continue; % Skip to next iteration
        end

        if isempty(prevGrayImage)
            % First frame: Initialize map and pose
            globalMap = currentPointCloud;
            poseLog = eye(4); % Initial pose
            visualizeSLAM(globalMap, poseLog);
        else
            % Feature Extraction and Matching
            [matchedPoints1, matchedPoints2] = extractAndMatchFeatures(prevGrayImage, enhancedGray);

            % Pose Estimation
            if ~isempty(matchedPoints1) && ~isempty(matchedPoints2)
                try
                    [R, t, inlierPts1, inlierPts2] = estimatePose(matchedPoints1, matchedPoints2, intrinsics);
                catch ME
                    warning('Pose estimation failed: %s', ME.message);
                    continue; % Skip to next iteration
                end

                % Update Map
                globalMap = updateMap(globalMap, currentPointCloud, R, t);

                % Track Path
                poseLog = trackPath(poseLog, R, t);

                % Visualize
                visualizeSLAM(globalMap, poseLog);
            else
                warning('No matched features found between frames.');
            end
        end

        % Update Previous Frame Variables
        prevGrayImage = enhancedGray;
        prevPointCloud = currentPointCloud;

        % Pause briefly to allow for GUI updates and reduce CPU load
        pause(0.01);
    end
catch ME
    disp('An error occurred during SLAM:');
    disp(ME.message);
end

% Cleanup
camera.stopCapture();
delete(camera);
disp('SLAM terminated and camera resources released.');
