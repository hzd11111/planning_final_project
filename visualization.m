%% Map Visualization
map_path = 'map3.txt';
original_map = readmatrix(map_path);

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(original_map'); axis square; colorbar; colormap jet; hold on;

%% Visualize Output
output_path = 'map_vis.txt';
output_map = readmatrix(output_path);

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(output_map'); axis square; colorbar; colormap jet; hold on;
