%% Map Visualization
map_path = 'map.txt';
original_map = readmatrix(map_path);

%draw the environment
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(original_map'); axis square; colorbar; colormap jet; hold on;