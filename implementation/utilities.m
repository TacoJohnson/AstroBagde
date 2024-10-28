% utilities.m
% Contains utility functions for the SLAM system

% Example: Function to convert rotation matrix and translation vector to a pose matrix
function poseMatrix = rtToPoseMatrix(R, t)
    % rtToPoseMatrix Converts rotation matrix and translation vector to a 4x4 pose matrix.
    %
    % Inputs:
    %   - R: Rotation matrix (3x3)
    %   - t: Translation vector (3x1)
    %
    % Output:
    %   - poseMatrix: 4x4 homogeneous transformation matrix

    poseMatrix = eye(4);
    poseMatrix(1:3,1:3) = R;
    poseMatrix(1:3,4) = t;
end
