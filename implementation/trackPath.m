% trackPath.m
% Function to track the camera's trajectory over time.

function poseLog = trackPath(poseLog, R, t)
    % trackPath Updates the pose log with the latest rotation and translation.
    %
    % Inputs:
    %   - poseLog: Existing pose log (4x4 matrix).
    %   - R: Rotation matrix (3x3).
    %   - t: Translation vector (3x1).
    %
    % Output:
    %   - poseLog: Updated pose log.

    % Create a homogeneous transformation matrix
    T = eye(4);
    T(1:3,1:3) = R;
    T(1:3,4) = t;

    % Update the pose log by concatenating the new pose
    poseLog = [poseLog, T];
end
