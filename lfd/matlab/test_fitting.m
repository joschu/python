
%% fitting
clear; close all
load('pointset_pair.mat')


opts = opts_fit;
opts.reg = 0.001;
params = tps_fit(xy1, xy2, opts);

xy2est = tps_eval(xy1,params);
clf
hold on
plot(x1, y1,'r')
plot(x2, y2,'g')
plot(xy2est(:,1), xy2est(:,2),'b')
plot_warped_grid(@(x) tps_eval(x,params))
