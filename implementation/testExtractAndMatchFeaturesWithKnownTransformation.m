% testExtractAndMatchFeaturesWithKnownTransformation.m
% Test script to verify feature extraction and pose estimation with known transformation.

clc; clear; close all;

% Create a base grayscale image with multiple distinct features
grayImage1 = uint8(zeros(172, 224));
grayImage1(50:60, 80:100) = 255;    % Bright rectangle
grayImage1(70:80, 120:140) = 180;   % Medium brightness rectangle
grayImage1(90:100, 160:180) = 220;  % Brighter rectangle
grayImage1(110:120, 200:220) = 160; % Another rectangle

% Define a known transformation (translation)
shiftAmount = 5; % pixels

% Create the second grayscale image by shifting the first image
grayImage2 = circshift(grayImage1, [0, shiftAmount]); % Shift right by 'shiftAmount' pixels

% Simulate depth maps (same depth for both images)
z1 = rand(172, 224) * 15; % Depth in meters
z2 = z1; % No depth change

% Load camera calibration data
S = load('cameraCalib.mat');

% Verify 'cameraParams' exists
if isfield(S, 'cameraParams')
    cameraParams = S.cameraParams;
    disp('cameraParams exists in cameraCalib.mat.');
else
    error('cameraCalib.mat does not contain "cameraParams".');
end

% Verify 'Intrinsics' property exists using isprop
if isprop(cameraParams, 'Intrinsics')
    intrinsics = cameraParams.Intrinsics;
    disp('Intrinsics property found in cameraParams.');
else
    error('cameraParams does not contain the "Intrinsics" property.');
end

% Display the Intrinsic Matrix
disp('Intrinsic Matrix:');
disp(intrinsics.IntrinsicMatrix);

% Generate consistent point clouds reflecting the image shift
[X, Y] = meshgrid(1:224, 1:172);
prevPointCloud = [X(:), Y(:), z1(:)];
pcFinal = [X(:) + shiftAmount, Y(:), z2(:)]; % Shift x-coordinate

% Call the feature extraction and matching function
[matchedPoints1, matchedPoints2, depth1, depth2] = extractAndMatchFeatures(grayImage1, grayImage2, prevPointCloud(:,3), pcFinal(:,3));

% Display results
disp(['Matched Points 1 Size: ', mat2str(size(matchedPoints1))]);
disp(['Matched Points 2 Size: ', mat2str(size(matchedPoints2))]);
disp(['Depth1 Size: ', mat2str(size(depth1))]);
disp(['Depth2 Size: ', mat2str(size(depth2))]);

% Verify that intrinsics is of the correct type
disp(['Intrinsics Class: ', class(intrinsics)]);

% Attempt pose estimation using MATLAB's built-in function
try
    % Convert matched points to [x y] format
    imagePoints = matchedPoints2; % Points in the second image
    
    % Define known rotation and translation (identity rotation, known translation)
    R = eye(3);                     % Known rotation matrix (identity)
    t = [shiftAmount; 0; 0];        % Known translation vector
    
    % Extract intrinsic parameters
    fx = intrinsics.FocalLength(1);
    fy = intrinsics.FocalLength(2);
    cx = intrinsics.PrincipalPoint(1);
    cy = intrinsics.PrincipalPoint(2);
    
    % Number of matched points
    N = size(matchedPoints1, 1);
    
    % Convert image points to camera coordinates
    Xc = (matchedPoints1(:,1) - cx) .* depth1 / fx;
    Yc = (matchedPoints1(:,2) - cy) .* depth1 / fy;
    Zc = depth1;
    
    % Assemble camera coordinates
    cameraPoints = [Xc, Yc, Zc]; % [N x 3]
    
    % Transform camera points to world coordinates
    worldPoints = (R * cameraPoints')' + repmat(t', N, 1); % [N x 3]
    
    % Estimate camera pose using built-in function
    [orientation, location, inlierWorldPoints, inlierImagePoints] = estimateWorldCameraPose(matchedPoints2, worldPoints, intrinsics);
    
    % Extract rotation matrix and translation vector from orientation
    R_est = orientation;              % Numeric rotation matrix
    t_est = location';                % Numeric translation vector (ensure column vector)
    
    % Display results
    disp('Pose estimation successful.');
    disp('Estimated Rotation Matrix:');
    disp(R_est);
    disp('Estimated Translation Vector:');
    disp(t_est);
    
    % Compare with expected translation
    expectedTranslation = t;          % Since we shifted horizontally
    disp('Expected Translation Vector:');
    disp(expectedTranslation);
catch ME
    disp('Pose estimation failed:');
    disp(ME.message);
end
