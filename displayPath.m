function a = displayPath(pose_filename, map_filename)
    if(isfile(pose_filename) && isfile(map_filename))
        output_path = map_filename;
        output_map = readmatrix(output_path);
        robot_path = pose_filename;
        robot_path = readmatrix(robot_path);

        %figure(1)
        
        imagesc(output_map'); axis square; colorbar; colormap jet; hold on;
        for i = 1:size(robot_path,1)
            hr = text(robot_path(i,2), robot_path(i,1), int2str(i), 'Color', 'y', 'FontWeight', 'bold', 'FontSize',14);
        end
        hold on;
        a = 1;
    else
        a = 0;
    end
    
end