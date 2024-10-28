% updateMap.m
% Function to update the global map with new point cloud data.

function globalMap = updateMap(globalMap, pcFinal, R, t)
    % updateMap Transforms and merges the new point cloud into the global map.
    %
    % Inputs:
    %   - globalMap: Existing global map point cloud.
    %   - pcFinal: New point cloud to add.
    %   - R: Rotation matrix.
    %   - t: Translation vector.
    %
    % Output:
    %   - globalMap: Updated global map point cloud.

    % Transform the new point cloud based on estimated pose
    transformedPoints = (R * pcFinal')' + repmat(t', size(pcFinal,1), 1);

    % Merge with the global map
    globalMap = [globalMap; transformedPoints];
end
