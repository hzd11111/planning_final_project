%% Map Visualization
map_path = 'map3.txt';
original_map = readmatrix(map_path);

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(original_map'); axis square; colorbar; colormap jet; hold on;

%% Visualize Output
output_path = 'map_vis_intermediate.txt';
output_map = readmatrix(output_path);
robot_path = "robot_poses.txt";
robot_path = readmatrix(robot_path);



%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(output_map'); axis square; colorbar; colormap jet; hold on;
for i = 1:size(robot_path,1)
    hr = text(robot_path(i,2), robot_path(i,1), int2str(i), 'Color', 'y', 'FontWeight', 'bold', 'FontSize',14);
end
hold off;

%% Output Sequentially
timestamp = 1;
foldername = "test_exp";
robot_pos_file = "/robot_poses";
map_file_name = "/map_vis_intermediate";
figure("Name", "Multi-Robot-Exploration", 'units','normalized','outerposition',[0 0 1 1]);
while 1
    if(displayPath(foldername+robot_pos_file+int2str(timestamp),foldername+map_file_name+int2str(timestamp)))
        timestamp = timestamp + 10;
    end
    pause(0.001);
end

