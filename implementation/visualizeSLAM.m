% visualizeSLAM.m
% Function to visualize the global map and camera trajectory.

function visualizeSLAM(globalMap, poseLog)
    % visualizeSLAM Updates the visualization of the global map and trajectory.
    %
    % Inputs:
    %   - globalMap: Global map point cloud.
    %   - poseLog: Pose log matrix (4xN).

    % Update Global Map Figure
    figureMap = findobj('Name', 'Global Map');
    if isempty(figureMap)
        figureMap = figure('Name', 'Global Map', 'NumberTitle', 'off');
        scatter3(globalMap(:,1), globalMap(:,2), globalMap(:,3), 1, 'b.');
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('Global Map');
        grid on;
        hold on;
    else
        hold on;
        scatter3(globalMap(end,1), globalMap(end,2), globalMap(end,3), 1, 'b.');
    end

    % Update Trajectory Figure
    figureTraj = findobj('Name', 'Trajectory');
    if isempty(figureTraj)
        figureTraj = figure('Name', 'Trajectory', 'NumberTitle', 'off');
        plot3(poseLog(1,1), poseLog(2,1), poseLog(3,1), 'ro-', 'LineWidth', 2);
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('Astronaut Trajectory');
        grid on;
        hold on;
    else
        plot3(poseLog(1,1:end), poseLog(2,1:end), poseLog(3,1:end), 'ro-', 'LineWidth', 2);
    end

    drawnow;
end
