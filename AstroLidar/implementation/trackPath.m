function poseLog = trackPath(poseLog, R, t)
    % trackPath Logs the current pose to the pose log.
    %
    % Inputs:
    %   - poseLog: Existing pose log (4x4xN)
    %   - R: Rotation matrix from current pose estimation (3x3)
    %   - t: Translation vector from current pose estimation (3x1)
    %
    % Output:
    %   - poseLog: Updated pose log (4x4xN)

    % Create pose matrix
    poseMatrix = eye(4);
    poseMatrix(1:3,1:3) = R;
    poseMatrix(1:3,4) = t;

    if isempty(poseLog)
        poseLog = poseMatrix;
    else
        % Accumulate pose: globalPose = previousPose * currentPose
        previousPose = poseLog(:,:,end);
        globalPose = previousPose * poseMatrix;
        poseLog(:,:,end+1) = globalPose;
    end
end
