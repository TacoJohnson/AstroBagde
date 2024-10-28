function camera = initializeCamera()
    % initializeCamera Initializes the Basler Flexx2 camera.
    % Returns a camera object ready for data capture.

    % Add path to Royale MATLAB SDK
    royalePath = 'C:\Program Files\royale\4.22.0.926\matlab'; % Update if necessary
    if exist(royalePath, 'dir') == 7
        addpath(royalePath);
    else
        error('Royale MATLAB SDK path not found. Please verify the installation.');
    end

    % Initialize Camera Manager
    manager = royale.CameraManager();
    camList = manager.getConnectedCameraList();
    if isempty(camList)
        error('No Flexx2 camera detected. Please check the connection.');
    end

    % Create and initialize camera
    camera = manager.createCamera(camList{1});
    camera.initialize();

    % Set Use Case
    useCases = camera.getUseCases();
    if length(useCases) >= 8
        camera.setUseCase(useCases{8});
    else
        error('Desired use case index not available.');
    end

    % Start Capture
    camera.startCapture();

    disp('Flexx2 camera initialized and capturing.');
end
