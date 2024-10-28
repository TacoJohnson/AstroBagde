% extractAndMatchFeatures.m
% Function to detect and match SURF features between two grayscale images and associate depth.

function [matchedPoints1, matchedPoints2, depth1, depth2] = extractAndMatchFeatures(grayImage1, grayImage2, z1, z2)
    % extractAndMatchFeatures Detects and matches SURF features between two grayscale images and associates depth.
    %
    % Inputs:
    %   - grayImage1: First grayscale image (uint8)
    %   - grayImage2: Second grayscale image (uint8)
    %   - z1: Depth map for the first image (vector, length = numPixels)
    %   - z2: Depth map for the second image (vector, length = numPixels)
    %
    % Outputs:
    %   - matchedPoints1: Matched points in the first image as [x, y] coordinates
    %   - matchedPoints2: Corresponding matched points in the second image as [x, y] coordinates
    %   - depth1: Depth values for matched points in the first image
    %   - depth2: Depth values for matched points in the second image

    % Validate inputs
    if ~isa(grayImage1, 'uint8') || ~isa(grayImage2, 'uint8')
        error('Grayscale images must be of type uint8.');
    end

    if ~isvector(z1) || ~isvector(z2)
        error('Depth inputs z1 and z2 must be vectors.');
    end

    % Detect SURF features
    points1_SURF = detectSURFFeatures(grayImage1, 'MetricThreshold', 10);
    points2_SURF = detectSURFFeatures(grayImage2, 'MetricThreshold', 10);

    % Extract SURF feature descriptors
    [features1_SURF, validPoints1_SURF] = extractFeatures(grayImage1, points1_SURF, 'Method', 'SURF');
    [features2_SURF, validPoints2_SURF] = extractFeatures(grayImage2, points2_SURF, 'Method', 'SURF');

    % Match SURF features using default 'SSD' metric
    indexPairs_SURF = matchFeatures(features1_SURF, features2_SURF, ...
        'MatchThreshold', 80, 'MaxRatio', 0.75); % 'Metric' parameter is omitted for default 'SSD'

    % Retrieve matched SURF points
    matchedPoints1_SURF = validPoints1_SURF(indexPairs_SURF(:,1), :);
    matchedPoints2_SURF = validPoints2_SURF(indexPairs_SURF(:,2), :);

    % Convert matched points to numeric [x, y] coordinates
    matchedPoints1_SURF_Loc = matchedPoints1_SURF.Location;
    matchedPoints2_SURF_Loc = matchedPoints2_SURF.Location;

    % Assign matched points
    matchedPoints1 = matchedPoints1_SURF_Loc;
    matchedPoints2 = matchedPoints2_SURF_Loc;

    % Associate depth with matched points
    % Ensure that x and y are within image bounds
    height = size(grayImage1, 1);
    width = size(grayImage1, 2);

    x1 = round(matchedPoints1(:,1));
    y1 = round(matchedPoints1(:,2));
    x2 = round(matchedPoints2(:,1));
    y2 = round(matchedPoints2(:,2));

    % Clamp coordinates to image dimensions
    x1 = max(min(x1, width), 1);
    y1 = max(min(y1, height), 1);
    x2 = max(min(x2, width), 1);
    y2 = max(min(y2, height), 1);

    % Convert (x, y) to linear indices
    linearIdx1 = sub2ind([height, width], y1, x1);
    linearIdx2 = sub2ind([height, width], y2, x2);

    % Debugging Statements
    disp(['Maximum linearIdx1: ', num2str(max(linearIdx1))]);
    disp(['Length of z1: ', num2str(length(z1))]);
    disp(['Maximum linearIdx2: ', num2str(max(linearIdx2))]);
    disp(['Length of z2: ', num2str(length(z2))]);

    % Ensure that linear indices do not exceed the size of z1 and z2
    if max(linearIdx1) > length(z1)
        error('linearIdx1 exceeds the length of z1.');
    end
    if max(linearIdx2) > length(z2)
        error('linearIdx2 exceeds the length of z2.');
    end

    % Retrieve depth values
    depth1 = z1(linearIdx1);
    depth2 = z2(linearIdx2);

    % Debugging Statements for Depth Values
    numNaNDepth1 = sum(isnan(depth1));
    numInfDepth1 = sum(isinf(depth1));
    numNaNDepth2 = sum(isnan(depth2));
    numInfDepth2 = sum(isinf(depth2));

    disp(['Number of NaNs in depth1: ', num2str(numNaNDepth1)]);
    disp(['Number of Infs in depth1: ', num2str(numInfDepth1)]);
    disp(['Number of NaNs in depth2: ', num2str(numNaNDepth2)]);
    disp(['Number of Infs in depth2: ', num2str(numInfDepth2)]);

    % Remove matches with invalid depth
    validDepth = (depth1 > 0) & (depth2 > 0) & ~isnan(depth1) & ~isnan(depth2) & ~isinf(depth1) & ~isinf(depth2);
    matchedPoints1 = matchedPoints1(validDepth, :);
    matchedPoints2 = matchedPoints2(validDepth, :);
    depth1 = depth1(validDepth);
    depth2 = depth2(validDepth);

    % Check if any matches are found
    if isempty(matchedPoints1) || isempty(matchedPoints2)
        matchedPoints1 = [];
        matchedPoints2 = [];
        depth1 = [];
        depth2 = [];
        warning('No valid feature matches found between frames.');
        return;
    end

    % Debugging Statements
    disp(['Total matches after filtering: ', num2str(length(depth1))]);

    % Optional: Visualize matched features (for debugging purposes)
    % figure;
    % showMatchedFeatures(grayImage1, grayImage2, matchedPoints1_SURF, matchedPoints2_SURF, 'montage');
    % title('Matched SURF Features with Depth');
end
