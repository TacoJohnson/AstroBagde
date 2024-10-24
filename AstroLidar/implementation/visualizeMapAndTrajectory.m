function visualizeMapAndTrajectory(globalMap, poseLog)
    % visualizeMapAndTrajectory Visualizes the global map and trajectory in real-time.
    %
    % Inputs:
    %   - globalMap: N-by-3 point cloud matrix
    %   - poseLog: M-by-4-by-4 pose matrices

    persistent mapFigure trajFigure mapHandle trajHandle

    if isempty(mapFigure) || ~ishandle(mapFigure)
        mapFigure = figure('Name', 'Global Map', 'NumberTitle', 'off');
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

    if isempty(trajFigure) || ~ishandle(trajFigure)
        trajFigure = figure('Name', 'Trajectory', 'NumberTitle', 'off');
        trajHandle = plot3(poseLog(:,4,1), poseLog(:,4,2), poseLog(:,4,3), 'r-', 'LineWidth', 2);
        xlabel('X (m)');
        ylabel('Y (m)');
        zlabel('Z (m)');
        title('Astronaut Trajectory');
        grid on;
        hold on;
    else
        set(trajHandle, 'XData', poseLog(:,4,1), 'YData', poseLog(:,4,2), 'ZData', poseLog(:,4,3));
    end

    drawnow limitrate;
end
