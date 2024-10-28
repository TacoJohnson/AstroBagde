% mainSLAMLoop.m
% Main script to run the SLAM system using Basler Flexx2 in MATLAB
% Tracks absolute rotation (roll, pitch, yaw) from the initial frame.

clc; clear; close all;

%% 1. Initialization

% Initialize Camera
camera = initializeCamera();

% Perform Camera Calibration (if not already calibrated)
if ~exist('cameraCalib.mat', 'file')
    disp('Camera calibration not found. Starting calibration...');
    calibrateCameraManual(0.025); % User provides square size in meters
    if ~exist('cameraCalib.mat', 'file')
        error('Calibration not completed. "cameraCalib.mat" not found.');
    end
end

% Load Camera Intrinsics
S = load('cameraCalib.mat'); % Load into structure 'S'

% Verify that 'cameraParams' exists and has 'Intrinsics'
if isfield(S, 'cameraParams') && isprop(S.cameraParams, 'Intrinsics')
    intrinsics = S.cameraParams.Intrinsics;
    disp('Loaded camera intrinsics from cameraCalib.mat.');
else
    error('Invalid calibration data. "cameraCalib.mat" must contain "cameraParams" with "Intrinsics".');
end

% Display the Intrinsic Matrix
disp('Intrinsic Matrix:');
disp(intrinsics.IntrinsicMatrix);

% Initialize Global Map and Pose Log
globalMap = [];
poseLog = [];

% Initialize Previous Frame Variables
prevGrayImage = [];
prevFilteredDepth = []; % Store previous depth map

% Initialize Variables for Initial Pose
R0 = []; % Initial rotation matrix
t0 = []; % Initial translation vector

% Initialize Figures for Visualization
% Global Map Figure
figureMap = figure('Name', 'Global Map', 'NumberTitle', 'off');
mapHandle = scatter3(0, 0, 0, 1, 'b.');
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Global Map');
grid on;
hold on;

% Trajectory Figure
figureTraj = figure('Name', 'Trajectory', 'NumberTitle', 'off');
trajHandle = plot3(0, 0, 0, 'r-', 'LineWidth', 2);
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Astronaut Trajectory');
grid on;
hold on;

% Initialize Text Annotation for Euler Angles
eulText = annotation('textbox', [0.15, 0.8, 0.3, 0.1], 'String', 'Total Rotation:\nRoll: 0°\nPitch: 0°\nYaw: 0°', ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'black', 'FontSize', 12);

% Initialize Cumulative Euler Angles Plots (Optional)
figureAngles = figure('Name', 'Cumulative Euler Angles', 'NumberTitle', 'off');

% Subplot for Roll
subplot(3,1,1);
plotRoll = plot(NaN, NaN, 'r-', 'LineWidth', 2);
title('Cumulative Roll (degrees)');
xlabel('Frame');
ylabel('Roll');
grid on;
hold on;

% Subplot for Pitch
subplot(3,1,2);
plotPitch = plot(NaN, NaN, 'g-', 'LineWidth', 2);
title('Cumulative Pitch (degrees)');
xlabel('Frame');
ylabel('Pitch');
grid on;
hold on;

% Subplot for Yaw
subplot(3,1,3);
plotYaw = plot(NaN, NaN, 'b-', 'LineWidth', 2);
title('Cumulative Yaw (degrees)');
xlabel('Frame');
ylabel('Yaw');
grid on;
hold on;

% Initialize Frame Counter
frameCount = 1;

%% 2. Main SLAM Loop

disp('Starting SLAM...');
try
    while ishandle(figureMap) && ishandle(figureTraj)
        % -----------------------------------
        % 2.1 Capture Data
        % -----------------------------------
        data = camera.getData();
        grayValue = []; x = []; y = []; z = [];

        % Assign Grayscale and Depth Data
        if isfield(data, 'grayValue') && isfield(data, 'x') && isfield(data, 'y') && isfield(data, 'z')
            grayValue = data.grayValue;
            x = data.x;
            y = data.y;
            z = data.z;
            disp('Captured new frame data.');
            disp(['grayValue Size: ', mat2str(size(grayValue))]);
            disp(['x Size: ', mat2str(size(x))]);
            disp(['y Size: ', mat2str(size(y))]);
            disp(['z Size: ', mat2str(size(z))]);
            disp(['Type of grayValue: ', class(grayValue)]);
        else
            warning('Necessary fields (grayValue, x, y, z) not found in data structure.');
            continue; % Skip to next iteration
        end

        % -----------------------------------
        % 2.2 Preprocess Data
        % -----------------------------------
        [filteredDepth, enhancedGray, pcFinal] = preprocessData(grayValue, x, y, z, intrinsics, 0.1);

        % Debugging Statements
        disp('Preprocessing complete.');
        disp(['Filtered Depth Size: ', mat2str(size(filteredDepth))]);
        disp(['Enhanced Gray Image Size: ', mat2str(size(enhancedGray))]);
        disp(['Point Cloud Size: ', mat2str(size(pcFinal))]);
        disp(['enhancedGray Type: ', class(enhancedGray)]); % Should now be uint8

        % Check if pcFinal is not empty
        if isempty(pcFinal)
            warning('Empty point cloud detected. Skipping frame.');
            continue; % Skip to next iteration
        end

        % -----------------------------------
        % 2.3 Handle First Frame Initialization
        % -----------------------------------
        if isempty(prevGrayImage)
            % First frame: Initialize map and pose
            globalMap = pcFinal;
            poseLog = eye(4); % Initial pose
            visualizeSLAM(globalMap, poseLog);
            disp('Initialized Global Map and Pose Log.');

            % Store current depth map for next iteration
            prevFilteredDepth = filteredDepth;

            % Assign enhancedGray to prevGrayImage
            prevGrayImage = enhancedGray;
            disp('Assigned enhancedGray to prevGrayImage.');

            % Store Initial Rotation and Translation
            R0 = eye(3); % Assuming initial pose is identity
            t0 = [0; 0; 0]; % Initial translation
            disp('Stored Initial Rotation (R0) and Translation (t0).');
        else
            % -----------------------------------
            % 2.4 Feature Extraction and Matching
            % -----------------------------------
            disp('Extracting and matching features...');
            [matchedPoints1, matchedPoints2, depth1, depth2] = extractAndMatchFeatures(prevGrayImage, enhancedGray, prevFilteredDepth(:), filteredDepth(:));
            disp(['Number of matched points: ', num2str(size(matchedPoints1,1))]);

            % -----------------------------------
            % 2.5 Pose Estimation
            % -----------------------------------
            if ~isempty(matchedPoints1) && ~isempty(matchedPoints2)
                try
                    % Extract intrinsic parameters
                    fx = intrinsics.FocalLength(1);
                    fy = intrinsics.FocalLength(2);
                    cx = intrinsics.PrincipalPoint(1);
                    cy = intrinsics.PrincipalPoint(2);

                    % Number of matched points
                    N = size(matchedPoints1, 1);

                    % Convert matched points to camera coordinates
                    Xc = (matchedPoints1(:,1) - cx) .* depth1 / fx;
                    Yc = (matchedPoints1(:,2) - cy) .* depth1 / fy;
                    Zc = depth1;

                    % Assemble camera coordinates
                    cameraPoints = [Xc, Yc, Zc]; % [N x 3]

                    % Define current pose (Assuming initial pose is identity)
                    % For subsequent frames, you would use the last pose to compute the new pose
                    % Here, we'll assume no initial movement for simplicity
                    R = eye(3);                     % Rotation matrix
                    t = [0; 0; 0];                   % Translation vector

                    % Transform camera points to world coordinates
                    worldPoints = (R * cameraPoints')' + repmat(t', N, 1); % [N x 3]

                    % Debugging Statements
                    disp(['Size of matchedPoints1: ', mat2str(size(matchedPoints1))]);
                    disp(['Size of matchedPoints2: ', mat2str(size(matchedPoints2))]);
                    disp(['Size of worldPoints: ', mat2str(size(worldPoints))]);

                    % Ensure that worldPoints has the same number of rows as matchedPoints2
                    if size(worldPoints, 1) ~= size(matchedPoints2, 1)
                        error('Mismatch between worldPoints and matchedPoints2 sizes.');
                    end

                    % Estimate camera pose using built-in function
                    [orientation, location, inlierWorldPoints, inlierImagePoints] = estimateWorldCameraPose(matchedPoints2, worldPoints, intrinsics);

                    % Extract rotation matrix and translation vector from orientation
                    R_est = orientation;              % Numeric rotation matrix
                    t_est = location';                % Numeric translation vector (ensure column vector)

                    % Compute Relative Rotation Matrix (R_rel = R0' * R_est)
                    R_rel = R0' * R_est;

                    % Compute Euler angles from Relative Rotation Matrix
                    if exist('rotm2eul', 'file')
                        eul_rel = rotm2eul(R_rel, 'XYZ'); % [roll; pitch; yaw] in radians
                        eul_rel_deg = rad2deg(eul_rel);
                    else
                        eul_rel_deg = rotm2eul_custom(R_rel); % [roll; pitch; yaw] in degrees
                    end
                    roll_rel_deg = eul_rel_deg(1);
                    pitch_rel_deg = eul_rel_deg(2);
                    yaw_rel_deg = eul_rel_deg(3);

                    % Display Euler angles
                    fprintf('Euler Angles (degrees): Roll = %.2f, Pitch = %.2f, Yaw = %.2f\n', roll_rel_deg, pitch_rel_deg, yaw_rel_deg);

                    % Update Euler angles display on figure
                    eulText.String = sprintf('Total Rotation:\nRoll: %.2f°\nPitch: %.2f°\nYaw: %.2f°', roll_rel_deg, pitch_rel_deg, yaw_rel_deg);

                    % -----------------------------------
                    % 2.6 Update Cumulative Euler Angles Plots
                    % -----------------------------------
                    figureAngles;
                    subplot(3,1,1);
                    set(plotRoll, 'XData', [get(plotRoll, 'XData'), frameCount], 'YData', [get(plotRoll, 'YData'), roll_rel_deg]);
                    hold on;

                    subplot(3,1,2);
                    set(plotPitch, 'XData', [get(plotPitch, 'XData'), frameCount], 'YData', [get(plotPitch, 'YData'), pitch_rel_deg]);
                    hold on;

                    subplot(3,1,3);
                    set(plotYaw, 'XData', [get(plotYaw, 'XData'), frameCount], 'YData', [get(plotYaw, 'YData'), yaw_rel_deg]);
                    hold on;

                    % Increment frame counter
                    frameCount = frameCount + 1;

                catch ME
                    warning(['Pose estimation failed: ', ME.message]);
                    continue; % Skip to next iteration
                end

                % -----------------------------------
                % 2.7 Update Global Map and Pose Log
                % -----------------------------------
                disp('Updating global map...');
                globalMap = updateMap(globalMap, pcFinal, R_est, t_est);
                disp(['Global map size after update: ', num2str(size(globalMap,1)), ' points.']);

                % Track Path
                poseLog = trackPath(poseLog, R_est, t_est);
                disp('Updated pose log.');

                % Visualize SLAM
                visualizeSLAM(globalMap, poseLog);
            else
                warning('No matched features found between frames.');
            end

            % -----------------------------------
            % 2.8 Update Previous Frame Variables
            % -----------------------------------
            prevGrayImage = enhancedGray;
            prevFilteredDepth = filteredDepth; % Update depth map
            prevPointCloud = pcFinal;

            % Debugging Statement
            disp('Updated prevGrayImage and prevFilteredDepth.');
            disp(['Size of prevGrayImage: ', mat2str(size(prevGrayImage))]);
        end
    end
    catch ME
        disp('An error occurred during SLAM:');
        disp(ME.message);
    end

    %% 3. Cleanup

    % Stop camera capture and release resources
    camera.stopCapture();
    delete(camera);
    disp('SLAM terminated and camera resources released.');

