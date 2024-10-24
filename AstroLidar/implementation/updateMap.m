function globalMap = updateMap(globalMap, pointCloud, R, t)
    % updateMap Transforms and integrates the current point cloud into the global map.
    %
    % Inputs:
    %   - globalMap: Existing global point cloud (N-by-3)
    %   - pointCloud: Current frame's point cloud (M-by-3)
    %   - R: Rotation matrix from pose estimation (3x3)
    %   - t: Translation vector from pose estimation (3x1)
    %
    % Output:
    %   - globalMap: Updated global point cloud (newN-by-3)

    if isempty(pointCloud)
        return;
    end

    % Apply transformation: p' = R * p + t
    transformedPC = (R * pointCloud')' + repmat(t', size(pointCloud,1), 1);

    % Concatenate with global map
    if isempty(globalMap)
        globalMap = transformedPC;
    else
        globalMap = [globalMap; transformedPC];
    end

    % Optional: Downsample global map to manage size
    globalMap = pcdownsample(pointCloud('Points', globalMap), 'gridAverage', 0.1).Location;
end
