function [R, t, inlierPoints1, inlierPoints2] = estimatePose(matchedPoints1, matchedPoints2, intrinsics)
    % estimatePose Estimates the relative rotation and translation between two sets of matched points.
    %
    % Inputs:
    %   - matchedPoints1: Matched points from the first image (vision.PointSet)
    %   - matchedPoints2: Matched points from the second image (vision.PointSet)
    %   - intrinsics: Camera intrinsic parameters object
    %
    % Outputs:
    %   - R: Rotation matrix (3x3)
    %   - t: Translation vector (3x1)
    %   - inlierPoints1: Inlier points from the first image
    %   - inlierPoints2: Inlier points from the second image

    % Extract matched points
    points1 = matchedPoints1.Location;
    points2 = matchedPoints2.Location;

    % Normalize points using intrinsics
    [normPoints1, ~] = normalizePoints(points1, intrinsics);
    [normPoints2, ~] = normalizePoints(points2, intrinsics);

    % Estimate Essential Matrix using RANSAC
    [E, inliers] = estimateEssentialMatrix(points1, points2, intrinsics, ...
        'Confidence', 99.99, 'MaxNumTrials', 1000);

    if isempty(E)
        error('Essential matrix estimation failed.');
    end

    % Extract inlier points
    inlierPoints1 = matchedPoints1(inliers, :);
    inlierPoints2 = matchedPoints2(inliers, :);

    % Recover Pose from Essential Matrix
    [R, t, ~] = recoverPose(E, inlierPoints1.Location, inlierPoints2.Location, intrinsics);
end

function [normalizedPoints, validIdx] = normalizePoints(points, intrinsics)
    % normalizePoints Normalizes image points using camera intrinsics.
    %
    % Inputs:
    %   - points: N-by-2 matrix of image points
    %   - intrinsics: Camera intrinsic parameters object
    %
    % Outputs:
    %   - normalizedPoints: N-by-2 matrix of normalized points
    %   - validIdx: Logical index of valid points

    fx = intrinsics.FocalLength(1);
    fy = intrinsics.FocalLength(2);
    cx = intrinsics.PrincipalPoint(1);
    cy = intrinsics.PrincipalPoint(2);

    normalizedPoints = [(points(:,1) - cx) / fx, (points(:,2) - cy) / fy];
    validIdx = ~isinf(normalizedPoints(:,1)) & ~isinf(normalizedPoints(:,2)) & ...
               ~isnan(normalizedPoints(:,1)) & ~isnan(normalizedPoints(:,2));
end
