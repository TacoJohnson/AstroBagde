% createCameraCalib.m
% Script to create and save camera calibration data correctly.

clc; clear; close all;

% Define intrinsic parameters
fx = 800; fy = 800;          % Focal lengths in pixels
cx = 86; cy = 112;           % Principal point coordinates in pixels
imageSize = [172, 224];      % Image size [Height, Width] in pixels

% Create cameraIntrinsics object with correct ordering
intrinsics = cameraIntrinsics([fx, fy], [cx, cy], imageSize);

% Create a structure to hold camera parameters
cameraParams = struct();
cameraParams.Intrinsics = intrinsics;

% Save to 'cameraCalib.mat'
save('cameraCalib.mat', 'cameraParams');

% Verify the saved Intrinsic Matrix
disp('Intrinsic Matrix saved to cameraCalib.mat:');
disp(intrinsics.IntrinsicMatrix);
