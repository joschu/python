function opts = opts_icp()

opts.plot_flag = 1;
opts.plot_grid = 0;
opts.lines_from_orig = 0;
opts.lines_from_mapped = 0;
opts.pause_each_iteration = 0;

opts.n_iter = 10;
opts.corr_opts = opts_corr();
opts.fit_opts = opts_fit();


end