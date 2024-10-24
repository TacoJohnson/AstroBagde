function [matchedPoints1, matchedPoints2] = extractAndMatchFeatures(grayImage1, grayImage2)
    % extractAndMatchFeatures Detects and matches ORB features between two grayscale images.
    %
    % Inputs:
    %   - grayImage1: First grayscale image (uint8)
    %   - grayImage2: Second grayscale image (uint8)
    %
    % Outputs:
    %   - matchedPoints1: Matched points in the first image
    %   - matchedPoints2: Corresponding matched points in the second image

    % Detect ORB features
    points1 = detectORBFeatures(grayImage1);
    points2 = detectORBFeatures(grayImage2);

    % Extract feature descriptors
    [features1, validPoints1] = extractFeatures(grayImage1, points1, 'Method', 'ORB');
    [features2, validPoints2] = extractFeatures(grayImage2, points2, 'Method', 'ORB');

    % Match features using BFMatcher with Hamming distance
    indexPairs = matchFeatures(features1, features2, 'MatchThreshold', 30, 'MaxRatio', 0.8);

    % Retrieve matched points
    matchedPoints1 = validPoints1(indexPairs(:,1), :);
    matchedPoints2 = validPoints2(indexPairs(:,2), :);

    % Optionally, visualize matches
    % figure;
    % showMatchedFeatures(grayImage1, grayImage2, matchedPoints1, matchedPoints2, 'montage');
    % title('Matched ORB Features');
end
