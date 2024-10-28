% verifyCameraCalib.m
% Script to verify the camera calibration data.

clc; clear; close all;

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
if isfield(cameraParams, 'Intrinsics')
    intrinsics = cameraParams.Intrinsics;
    disp('Intrinsics property found in cameraParams.');
else
    error('cameraParams does not contain the "Intrinsics" property.');
end

% Display the Intrinsic Matrix
disp('Intrinsic Matrix:');
disp(intrinsics.IntrinsicMatrix);
