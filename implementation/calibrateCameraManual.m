function calibrateCameraManual(squareSize)
    % calibrateCameraManual Guides the user through camera calibration using MATLAB's cameraCalibrator app.
    %
    % Usage:
    %   calibrateCameraManual()               % Uses default square size of 2.5 cm
    %   calibrateCameraManual(squareSize)     % Specifies square size in meters
    
    % Check and set default square size if not provided
    if nargin < 1
        squareSize = 0.03175; % Default square size: 2.5 cm
        disp(['No square size provided. Using default square size of ', num2str(squareSize*100), ' cm.']);
    else
        % Validate the provided square size
        if ~isnumeric(squareSize) || squareSize <= 0
            error('Square size must be a positive numeric value representing meters.');
        end
        disp(['Using provided square size of ', num2str(squareSize*100), ' cm.']);
    end
    
    % Initialize Camera
    camera = initializeCamera();
    
    % Collect Calibration Images
    numImages = 20; % Number of calibration images to capture
    imagesDir = 'C:\AstroLidar\implementation\CalibrationImages\'; % Update path as needed
    if ~exist(imagesDir, 'dir')
        mkdir(imagesDir);
    end
    
    disp('Starting camera calibration...');
    disp(['Please capture ', num2str(numImages), ' images of the checkerboard.']);
    disp('Ensure the checkerboard is clearly visible and occupies a significant portion of the frame.');
    
    for i = 1:numImages
        disp(['Capture image ', num2str(i), ' of ', num2str(numImages), '...']);
        % Optional: Add a delay or wait for user input to capture the image
        pause(2); % Wait for 2 seconds before capturing the image
        
        data = camera.getData();
        
        % Access the Grayscale Image
        if isfield(data, 'grayValue')
            grayImage = data.grayValue;
        else
            error('grayValue field not found in the data structure.');
        end
        
        % Convert grayValue to uint8 for processing
        grayImageUint8 = uint8(255 * mat2gray(grayImage));
        
        % Save image for calibration
        imageFileName = fullfile(imagesDir, ['calib_', sprintf('%02d', i), '.png']);
        imwrite(grayImageUint8, imageFileName);
        disp(['Image ', num2str(i), ' captured and saved as ', imageFileName, '.']);
    end
    
    % Stop Camera Capture Temporarily
    camera.stopCapture();
    
    % Verify Calibration Images
    imageFiles = dir(fullfile(imagesDir, '*.png'));
    if isempty(imageFiles)
        error('No calibration images found in the directory: %s', imagesDir);
    else
        disp(['Found ', num2str(length(imageFiles)), ' calibration images.']);
    end
    
    % Launch Camera Calibrator with Correct Arguments
    disp('Launching Camera Calibrator app...');
    try
        % Ensure imagesDir is a character vector and use forward slashes
        cameraCalibrator(char(imagesDir), squareSize);
    catch ME
        disp('Failed to launch Camera Calibrator:');
        disp(ME.message);
        disp('Please try launching the Camera Calibrator app manually.');
        % Optionally, provide instructions to manually launch
        disp(['To manually launch, type: cameraCalibrator(''', imagesDir, ''', ', num2str(squareSize), ');']);
    end
    
    % After calibration is done manually, save the calibration results
    % Users should save the calibration to 'cameraCalib.mat' from the Camera Calibrator app
    
    % Cleanup
    delete(camera);
    disp('Camera calibration process completed. Please ensure you have saved the calibration parameters as "cameraCalib.mat" from the Camera Calibrator app.');
end
