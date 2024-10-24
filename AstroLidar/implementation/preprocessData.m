function [filteredDepth, enhancedGray, pointCloud] = preprocessData(grayValue, x, y, z, intrinsics, voxelSize)
    % preprocessData Filters and enhances grayscale and depth data.
    % Generates a 3D point cloud from the depth data.
    %
    % Inputs:
    %   - grayValue: Grayscale image captured from the camera (uint16)
    %   - x, y, z: 3D coordinate matrices (single)
    %   - intrinsics: Camera intrinsic parameters object
    %   - voxelSize: Size of voxels for downsampling (in meters)
    %
    % Outputs:
    %   - filteredDepth: Processed depth map
    %   - enhancedGray: Enhanced grayscale image
    %   - pointCloud: Downsampled 3D point cloud

    % Convert grayValue to uint8 for processing
    grayImage = uint8(255 * mat2gray(grayValue));

    % Enhance Grayscale Image
    enhancedGray = imadjust(grayImage); % Adjust intensity values for better feature detection

    % Compute Depth Map
    % Assuming z is in meters; adjust if necessary
    depthMap = double(z);

    % Depth Map Filtering
    minDepthThreshold = 0.5; % meters
    maxDepthThreshold = 10.0; % meters
    filteredDepth = medfilt2(depthMap, [3 3]); % Median filter with 3x3 kernel
    filteredDepth(filteredDepth < minDepthThreshold | filteredDepth > maxDepthThreshold) = NaN;

    % Point Cloud Generation from x, y, z
    pointCloud = [x(:), y(:), z(:)];
    pointCloud(any(isnan(pointCloud), 2), :) = []; % Remove invalid points

    % Voxel Downsampling
    if ~isempty(pointCloud)
        pcObj = pointCloud('Points', pointCloud);
        downsampledPC = pcdownsample(pcObj, 'gridAverage', voxelSize);
        pointCloud = downsampledPC.Location;
    else
        pointCloud = [];
    end
end
