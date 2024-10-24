function visualizeSLAM(globalMap, poseLog)
    % visualizeSLAM Displays the global map and the astronaut's trajectory.
    %
    % Inputs:
    %   - globalMap: Global point cloud (N-by-3)
    %   - poseLog: Log of poses (4x4xN)
    %
    % Outputs:
    %   - None (visualization is handled via figures)

    persistent mapHandle trajHandle

    % Initialize or update the global map figure
    if isempty(mapHandle) || ~ishandle(mapHandle)
        figure(1);
        mapHandle = scatter3(globalMap(:,1), globalMap(:,2), globalMap(:,3), 1, 'b.');
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('Global Map');
        grid on;
        hold on;
    else
        set(mapHandle, 'XData', globalMap(:,1), 'YData', globalMap(:,2), 'ZData', globalMap(:,3));
    end

    % Initialize or update the trajectory figure
    if isempty(trajHandle) || ~ishandle(trajHandle)
        figure(2);
        trajHandle = plot3(0, 0, 0, 'r-', 'LineWidth', 2);
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('Astronaut Trajectory');
        grid on;
        hold on;
    else
        % Extract trajectory points
        trajX = squeeze(poseLog(:,4,1));
        trajY = squeeze(poseLog(:,4,2));
        trajZ = squeeze(poseLog(:,4,3));

        set(trajHandle, 'XData', trajX, 'YData', trajY, 'ZData', trajZ);
    end

    drawnow limitrate;
end
