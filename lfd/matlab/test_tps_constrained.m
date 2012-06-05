%% with plane
clear; close all
load('pointset_pair.mat')

opts = opts_fit;
plane_par = [1,0,-.6];
% params = tps_fit_constrained(xy1, xy2, xy1 + randn(size(xy1))*.001, plane_par, opts);
params = tps_fit(xy1, xy2, opts);

xy2est = tps_eval(xy1,params);
clf
hold on
plot(x1, y1,'r')
plot(x2, y2,'g')
plot(xy2est(:,1), xy2est(:,2),'bx')

