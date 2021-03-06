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
foldername = "9_robots";
makeVideo = 1;
if (makeVideo)
    writerObj = VideoWriter(foldername+'.avi');
    open(writerObj);
end

timestamp = 1;
robot_pos_file = "/robot_poses";
map_file_name = "/map_vis_intermediate";
figure("Name", "Multi-Robot-Exploration", 'units','normalized','outerposition',[0 0 1 1]);
while 1
    if(displayPath(foldername+robot_pos_file+int2str(timestamp),foldername+map_file_name+int2str(timestamp)))
        timestamp = timestamp + 10;
        frame = getframe;
        if (makeVideo)
            writeVideo(writerObj, frame);
        end
    elseif (makeVideo)
        break;
    end
    %pause(0.001);
end
% close the writer object
if(makeVideo)
    close(writerObj);
end

%% compress map

original_map = readmatrix("map3.txt");
%original_map = original_map(:,150:end);
new_map = zeros(int32(size(original_map,1)/4), int32(size(original_map,2)/4));
for i = 1:size(new_map,1)-1
    for j = 1:size(new_map,2)-1
        new_map(i,j) = original_map(4*i, 4*j);
    end
end
%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(new_map'); axis square; colorbar; colormap jet; hold on;
csvwrite('map_compressed.txt', new_map);
