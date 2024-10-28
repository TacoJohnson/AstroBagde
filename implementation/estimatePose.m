% estimatePose.m
% Function to estimate rotation and translation between two sets of matched points.

function [R, t, inlierPoints1, inlierPoints2] = estimatePose(matchedPoints1, matchedPoints2, intrinsics)
    % estimatePose Estimates the rotation and translation between two sets of matched points.
    %
    % Inputs:
    %   - matchedPoints1: Matched points in the first image as [x, y] coordinates
    %   - matchedPoints2: Corresponding matched points in the second image as [x, y] coordinates
    %   - intrinsics: Camera intrinsic parameters (cameraIntrinsics object)
    %
    % Outputs:
    %   - R: Rotation matrix (3x3)
    %   - t: Translation vector (3x1)
    %   - inlierPoints1: Inlier matched points from the first image as [x, y] coordinates
    %   - inlierPoints2: Corresponding inlier matched points from the second image as [x, y] coordinates

    % Validate input types
    if ~isa(intrinsics, 'cameraIntrinsics') && ~isa(intrinsics, 'cameraParameters')
        error('Invalid intrinsics type. Expected cameraIntrinsics or cameraParameters, got %s.', class(intrinsics));
    end

    % Extract camera matrix from intrinsics
    cameraMatrix = intrinsics.IntrinsicMatrix';

    % Check matched points
    if size(matchedPoints1,1) < 5 || size(matchedPoints2,1) < 5
        error('Not enough matched points for Essential Matrix estimation.');
    end

    % Estimate Essential Matrix using RANSAC
    try
        [E, inliers] = estimateEssentialMatrix(matchedPoints1, matchedPoints2, cameraMatrix, ...
            'Confidence', 99.99, 'MaxNumTrials', 1000);
    catch ME
        error('Essential matrix estimation failed: %s', ME.message);
    end

    if isempty(E)
        error('Essential matrix estimation returned empty.');
    end

    % Extract inlier points
    inlierPoints1 = matchedPoints1(inliers, :);
    inlierPoints2 = matchedPoints2(inliers, :);

    % Check if enough inliers are found
    minInliers = 5; % Minimum number of inliers required
    if size(inlierPoints1,1) < minInliers
        error('Not enough inlier matches after RANSAC. Required: %d, Found: %d', minInliers, size(inlierPoints1,1));
    end

    % Recover Pose from Essential Matrix
    try
        [R, t, ~] = recoverPose(E, inlierPoints1, inlierPoints2, cameraMatrix);
    catch ME
        error('Pose recovery failed: %s', ME.message);
    end

    % Verify the outputs
    if isempty(R) || isempty(t)
        error('Pose recovery returned empty results.');
    end
end
