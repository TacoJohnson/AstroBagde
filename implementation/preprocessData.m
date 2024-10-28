% preprocessData.m
% Function to preprocess captured grayscale and depth data.

function [filteredDepth, enhancedGray, pcFinal] = preprocessData(grayValue, X, Y, Z, intrinsics, depthThreshold)
    % preprocessData Preprocesses grayscale and depth data.
    %
    % Inputs:
    %   - grayValue: Raw grayscale image data.
    %   - X: X-coordinates array (pixel indices)
    %   - Y: Y-coordinates array (pixel indices)
    %   - Z: Z-coordinates (depth) array.
    %   - intrinsics: Camera intrinsic parameters (cameraIntrinsics object).
    %   - depthThreshold: Depth filtering threshold in meters.
    %
    % Outputs:
    %   - filteredDepth: Filtered depth data [Height x Width].
    %   - enhancedGray: Enhanced grayscale image [Height x Width], uint8.
    %   - pcFinal: Final point cloud in camera coordinates [Nx3].

    % 1. Filter depth based on threshold
    filteredDepth = Z;
    filteredDepth(Z < depthThreshold) = NaN;

    % 2. Enhance grayscale image (e.g., histogram equalization)
    enhancedGray = histeq(grayValue);

    % 3. Convert enhancedGray to uint8 if it's not already
    if ~isa(enhancedGray, 'uint8')
        % Normalize the image to [0, 1] and then scale to [0, 255]
        enhancedGray = im2uint8(mat2gray(enhancedGray));
        disp('Converted enhancedGray to uint8.');
    end

    % 4. Generate point cloud
    % Reshape X, Y, Z to 2D arrays if necessary
    X = reshape(X, size(grayValue));
    Y = reshape(Y, size(grayValue));
    Z = reshape(filteredDepth, size(grayValue));

    % Flatten the matrices for processing
    X_flat = X(:);
    Y_flat = Y(:);
    Z_flat = Z(:);

    % Remove points with NaN depth
    validIdx = ~isnan(Z_flat) & Z_flat > 0;
    X_valid = X_flat(validIdx);
    Y_valid = Y_flat(validIdx);
    Z_valid = Z_flat(validIdx);

    % Extract intrinsic parameters
    fx = intrinsics.FocalLength(1);
    fy = intrinsics.FocalLength(2);
    cx = intrinsics.PrincipalPoint(1);
    cy = intrinsics.PrincipalPoint(2);

    % Convert to camera coordinates
    Xc = (X_valid - cx) .* Z_valid / fx;
    Yc = (Y_valid - cy) .* Z_valid / fy;
    Zc = Z_valid;

    % Assemble point cloud in camera coordinates
    pcFinal = [Xc, Yc, Zc];
end
