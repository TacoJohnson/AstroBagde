% testEstimatePose.m

% Simulated Matched Points
matchedPoints1 = [100, 150; 120, 160; 130, 170; 140, 180; 150, 190; 160, 200];
matchedPoints2 = [102, 152; 122, 162; 132, 172; 142, 182; 152, 192; 162, 202];

% Simulated Intrinsics (cameraIntrinsics object)
fx = 800; fy = 800;
cx = 86; cy = 112;
intrinsics = cameraIntrinsics([cx, cy], [fx, fy], [172, 224]);

% Call the pose estimation function
try
    [R, t, inlierPts1, inlierPts2] = estimatePose(matchedPoints1, matchedPoints2, intrinsics);
    disp('Pose estimation successful.');
    disp('Rotation Matrix:');
    disp(R);
    disp('Translation Vector:');
    disp(t);
catch ME
    disp('Pose estimation failed:');
    disp(ME.message);
end
