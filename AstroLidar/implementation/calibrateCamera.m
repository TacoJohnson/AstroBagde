function intrinsics = calibrateCamera()
    % calibrateCamera Performs camera calibration and returns intrinsic parameters.
    %
    % Output:
    %   - intrinsics: Camera intrinsic parameters object

    % Specify checkerboard pattern size
    squareSize = 0.025; % 2.5 cm, adjust based on actual checkerboard

    % Create cameraCalibrator object
    camCalib = cameraCalibrator('SquareSize', squareSize);

    % Initialize camera
    camera = initializeCamera();

    % Collect calibration images
    disp('Collecting calibration images. Press "Enter" to capture each image.');

    for i = 1:20
        pause; % Wait for user to press Enter
        data = camera.getData();
        rgbImage = data.rgbImage;

        % Detect checkerboard points
        [imagePoints, boardSize] = detectCheckerboardPoints(rgbImage);

        if ~isempty(imagePoints)
            % Add image and detected points to calibration
            camCalib.addImage(rgbImage, imagePoints);
            disp(['Captured image ' num2str(i) ' with detected checkerboard points.']);
        else
            disp(['Image ' num2str(i) ' does not contain a detectable checkerboard. Skipping.']);
        end
    end

    % Run calibration
    [cameraParams, imagesUsed, estimationErrors] = calibrate(camCalib);

    % Display calibration results
    showReprojectionErrors(cameraParams);
    figure;
    showExtrinsics(cameraParams, 'CameraCentric');

    % Save calibration parameters
    save('cameraCalib.mat', 'cameraParams');

    % Assign intrinsics
    intrinsics = cameraParams.Intrinsics;

    % Cleanup
    camera.stopCapture();
    delete(camera);
    disp('Camera calibration completed and intrinsics saved.');
end
