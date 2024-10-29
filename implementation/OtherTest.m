clc; clear;

% Add path to the Royale MATLAB SDK (change the path based on your installation)
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

% Parameters for basic motion model (assumed small motion steps for mapping)
theta = 0;  % Initialize rotation angle
translation = [0; 0];  % Initialize translation

% Create a figure for real-time visualization
figure;
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');

% Number of point cloud scans
numScans = 50;

% Loop to capture point clouds and simulate mapping
for i = 1:numScans
    % Capture the point cloud
    [~, PointCloudData] = cameraDevice.getData();
    
    % Get XYZ coordinates of the point cloud
    ptCloud = pointCloud(PointCloudData);  % Create a point cloud object
    
    % Check if point cloud is valid before processing
    if isempty(ptCloud.Location)
        warning('No point cloud data captured in scan %d', i);
        continue;
    end
    
    % Extract X and Y coordinates for 2D mapping
    xyPoints = ptCloud.Location(:, 1:2);
    
    % Apply motion model: translate and rotate the point cloud (assumed motion)
    % Modify 'theta' and 'translation' as per the expected movement
    rotationMatrix = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    transformedPoints = (rotationMatrix * xyPoints')' + translation';
    
    % Plot the transformed point cloud
    plot(transformedPoints(:,1), transformedPoints(:,2), '.');
    title(['Mapping Process - Scan ' num2str(i)]);
    drawnow;
    
    % Update motion parameters (example: small rotation and translation steps)
    theta = theta + 0.02;  % Incremental rotation (radians)
    translation = translation + [0.05; 0];  % Incremental translation (meters)
end

% Stop capturing
cameraDevice.stopCapture();

disp('Mapping process completed.');
