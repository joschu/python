%% make two point sets
clf
close all
clear


figure(1)
axis([0,1,0,1])
[x1,y1] = getpts;
xy1 = [x1,y1];

clf
axis([0,1,0,1])
[x2,y2] = getpts;
xy2 = [x2,y2];

% save('pointset_pair.mat', 'xy1', 'xy2', 'x1', 'y1', 'x2', 'y2')
save('diff_size_points.mat', 'xy1', 'xy2', 'x1', 'y1', 'x2', 'y2')
% load('pointset_pair.mat')