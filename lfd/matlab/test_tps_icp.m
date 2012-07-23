




clear; close all; clf
rope0 = load('~/Data/rope/rope1.txt','-ascii');
simrope0 = load('~/Data/rope/simrope1.txt','-ascii')*diag([1,-1,1]);
%

clf
hold on
plot(rope0(:,1), rope0(:,2),'ro')
plot(simrope0(:,1), simrope0(:,2),'bo')
% plot(xy2est(:,1), xy2est(:,2),'b')
%%
opts = opts_icp;
opts.n_iter = 6;
opts.lines_from_orig=1;
opts.corr_opts.method='bipartite';
opts.fit_opts.reg = .01;
opts.plot_grid = 1;
opts.corr_opts.bipartite_opts.deficient_col_penalty=1;
opts.corr_opts.bipartite_opts.deficient_row_penalty=1;
tps_rpm(simrope0(:,1:2), rope0(1:10:end,1:2),opts)%s,.1*simrope0(:,4:end), .1*rope0(1:10:end,4:end))
