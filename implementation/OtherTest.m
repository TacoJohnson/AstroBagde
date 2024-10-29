clc; clear;

% Add path to the Royale MATLAB SDK
addpath('C:\Program Files\royale\4.22.0.926\matlab');

% Initialize the Camera and Check Connection
manager = royale.CameraManager();
camlist = manager.getConnectedCameraList();
if isempty(camlist)
    error('No camera found. Please check the connection.');
end
cameraDevice = manager.createCamera(camlist{1});
cameraDevice.initialize();

% Set a Use Case
UseCases = cameraDevice.getUseCases();
if isempty(UseCases)
    error('No use case available');
end

% Set the use case to the 8th available as per your setup
if length(UseCases) >= 8
    cameraDevice.setUseCase(UseCases{8});
else
    error('The specified use case index is not available.');
end

% Start capturing
cameraDevice.startCapture();

% Initialize variables
globalMap = pointCloud(zeros(0,3)); % Corrected initialization
prevPointCloud = [];
prevImage = [];
prevFeatures = [];
prevPoints = [];
prevPose = rigid3d(); % Initial pose (identity)

% Parameters for voxelization
voxelSize = 0.1; % Voxel size in meters
mapLimits = [-5, 5;   % X limits in meters
             -1, 3;   % Y limits in meters (vertical axis)
             -5, 5];  % Z limits in meters

mapSizeX = mapLimits(1,2) - mapLimits(1,1); % Total size in X
mapSizeY = mapLimits(2,2) - mapLimits(2,1); % Total size in Y
mapSizeZ = mapLimits(3,2) - mapLimits(3,1); % Total size in Z

gridSizeX = ceil(mapSizeX / voxelSize);
gridSizeY = ceil(mapSizeY / voxelSize);
gridSizeZ = ceil(mapSizeZ / voxelSize);

% Initialize voxel grid
voxelGrid = zeros(gridSizeX, gridSizeY, gridSizeZ);

% Create figure for visualization
hFig = figure;

% Left subplot for 3D point cloud
hAxes = subplot(1,2,1);
xlabel(hAxes, 'X (meters)');
ylabel(hAxes, 'Y (meters)');
zlabel(hAxes, 'Z (meters)');
title(hAxes, 'Accumulated Point Cloud Map');
grid(hAxes, 'on');
axis(hAxes, 'equal');
hold(hAxes, 'on');

% Right subplot for top-down map (X-Z view)
hTopDownAxes = subplot(1,2,2);
xlabel(hTopDownAxes, 'X (meters)');
ylabel(hTopDownAxes, 'Z (meters)');
title(hTopDownAxes, 'Top-down Voxelized Map (X-Z View)');
axis(hTopDownAxes, 'equal');
hold(hTopDownAxes, 'on');

% Camera intrinsics using cameraIntrinsics
focalLength = [210.7, 213.2]; % Calculated focal lengths in pixels
principalPoint = [112, 86];    % Image center
imageSize = [172, 224];        % [height, width]

intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

% Main loop
while ishandle(hFig)
    % Fetch a single frame
    data = cameraDevice.getData();
    points3D = [data.x(:), data.y(:), data.z(:)];
    grayImage = data.grayValue; % Get grayscale image

    % Determine the correct image dimensions
    [height, width] = size(data.z); % Assuming data.z is 2D

    % Reshape the grayscale image
    grayImage = reshape(grayImage, [height, width]);
    grayImage = uint8(grayImage); % Convert to uint8 for feature extraction

    % Image preprocessing to enhance features
    grayImage = adapthisteq(grayImage);       % Adaptive histogram equalization
    grayImage = imgaussfilt(grayImage, 1);    % Gaussian filtering to reduce noise

    % Optionally display the grayscale image for verification
    % figure;
    % imshow(grayImage, []);
    % title('Enhanced Grayscale Image from Camera');

    validPoints = points3D(:, 3) > 0; % Filter valid points based on depth
    points3D = points3D(validPoints, :);

    % Invert Y-axis data to correct the visualization
    points3D(:,2) = -points3D(:,2);

    % Create point cloud object
    currPointCloud = pointCloud(points3D);

    % Preprocess point cloud (e.g., downsample)
    currPointCloud = pcdownsample(currPointCloud, 'gridAverage', 0.2); % Downsample with 2 cm grid

    % Extract features from the grayscale image using KAZE features
    currImage = grayImage;
    currPoints = detectKAZEFeatures(currImage, 'Threshold', 0.001);
    currPoints = currPoints.selectStrongest(2000);
    [currFeatures, currValidPoints] = extractFeatures(currImage, currPoints);

    % Display number of features detected
    fprintf('Number of features detected: %d\n', currPoints.Count);

    % If this is the first frame
    if isempty(prevPointCloud)
        % Initialize global map and pose
        globalMap = currPointCloud;
        prevPointCloud = currPointCloud;
        prevImage = currImage;
        prevFeatures = currFeatures;
        prevPoints = currValidPoints;
        prevPose = rigid3d(); % Identity transformation
        continue; % Move to next iteration
    end

    % Match features between current and previous frames
    indexPairs = matchFeatures(prevFeatures, currFeatures, 'MatchThreshold', 100, 'MaxRatio', 0.9, 'Unique', false);

    matchedPoints1 = prevPoints(indexPairs(:,1));
    matchedPoints2 = currValidPoints(indexPairs(:,2));

    % Display number of matched features
    fprintf('Number of matched features: %d\n', size(indexPairs, 1));

    % Visualize matched features (optional)
    % figure;
    % showMatchedFeatures(prevImage, currImage, matchedPoints1, matchedPoints2);
    % title('Matched Features');

    % Estimate the relative pose using matched feature points
    if size(matchedPoints1, 1) >= 50
        % Estimate the essential matrix with camera intrinsics
        [E, inlierIdx] = estimateEssentialMatrix(matchedPoints1, matchedPoints2, intrinsics, 'Confidence', 99.99);

        % Recover relative camera pose
        if ~isempty(E) && numel(inlierIdx) >= 8
            [relativeOrient, relativeLoc, validIdx] = relativeCameraPose(E, intrinsics, ...
                matchedPoints1(inlierIdx), matchedPoints2(inlierIdx));

            if ~isempty(relativeOrient)
                % Convert rotation and translation to transformation matrix
                tform = rigid3d(relativeOrient, relativeLoc);

                % Update pose
                currPose = rigid3d(tform.T * prevPose.T);

                % Transform current point cloud to global coordinate frame
                alignedPointCloud = pctransform(currPointCloud, currPose);

                % Merge current point cloud into global map
                globalMap = pcmerge(globalMap, alignedPointCloud, 0.01); % Merge with 1 cm grid size

                % Update voxel grid
                voxelGrid = updateVoxelGrid(alignedPointCloud, voxelGrid, mapLimits, voxelSize);

                % Update previous data
                prevPointCloud = currPointCloud;
                prevImage = currImage;
                prevFeatures = currFeatures;
                prevPoints = currValidPoints;
                prevPose = currPose;
            else
                disp('Failed to recover relative camera pose.');
            end
        else
            disp('Not enough inlier matches to estimate essential matrix.');
        end
    else
        disp('Not enough matched points.');
    end

    % Generate top-down map by summing along Y-axis (vertical axis)
    topDownMap = squeeze(sum(voxelGrid, 2)); % Sum over Y-axis

    % Update visualization
    cla(hAxes);
    pcshow(globalMap, 'Parent', hAxes);
    xlabel(hAxes, 'X (meters)');
    ylabel(hAxes, 'Y (meters)');
    zlabel(hAxes, 'Z (meters)');
    title(hAxes, 'Accumulated Point Cloud Map');
    grid(hAxes, 'on');
    axis(hAxes, 'equal');
    hold(hAxes, 'on');

    cla(hTopDownAxes);
    imagesc(hTopDownAxes, [mapLimits(1,1), mapLimits(1,2)], [mapLimits(3,1), mapLimits(3,2)], flipud(topDownMap'));
    colormap(hTopDownAxes, 'gray');
    set(hTopDownAxes, 'YDir', 'normal');
    axis(hTopDownAxes, 'equal');
    xlabel(hTopDownAxes, 'X (meters)');
    ylabel(hTopDownAxes, 'Z (meters)');
    title(hTopDownAxes, 'Top-down Voxelized Map (X-Z View)');
    drawnow;
end

% Stop capturing and clean up
cameraDevice.stopCapture();
delete(cameraDevice);
delete(manager);

% Helper function to update voxel grid
function voxelGrid = updateVoxelGrid(alignedPointCloud, voxelGrid, mapLimits, voxelSize)
    globalPoints = alignedPointCloud.Location;

    % Convert points to voxel indices
    voxelIndices = floor((globalPoints - [mapLimits(1,1), mapLimits(2,1), mapLimits(3,1)]) / voxelSize) + 1;

    gridSizeX = size(voxelGrid, 1);
    gridSizeY = size(voxelGrid, 2);
    gridSizeZ = size(voxelGrid, 3);

    % Filter points within map limits
    validIdx = all(voxelIndices >= 1 & voxelIndices <= [gridSizeX, gridSizeY, gridSizeZ], 2);
    voxelIndices = voxelIndices(validIdx, :);

    % Voxelization: Mark the voxels that contain points
    linearIndices = sub2ind([gridSizeX, gridSizeY, gridSizeZ], voxelIndices(:,1), voxelIndices(:,2), voxelIndices(:,3));
    voxelGrid(linearIndices) = 1;
end
