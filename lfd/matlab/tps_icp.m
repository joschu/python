function params = tps_rpm(x_nd, y_md, opts, xfeats_nk, yfeats_mk)    

if nargin <= 3
    xfeats_nk = [];
    yfeats_mk = [];
end

if opts.plot_flag
    clf;
    hold on;
    d = size(x_nd,2);
    d
    if d ~= 2 && opts.plot_grid
        error('can only plot grid lines with d=2')
    end
%     if d==2
        plot(x_nd(:,1), x_nd(:,2), 'ro', 'LineWidth',2)
        plot(y_md(:,1), y_md(:,2), 'bo', 'LineWidth', 2)
%     end
%     if d==3
%         plot3(x_nd(:,1), x_nd(:,2), x_nd(:,3), 'ro', 'LineWidth',2)
%         plot3(y_md(:,1), y_md(:,2), y_md(:,3), 'bo', 'LineWidth', 2)
%     end    
    
    h_pred = [];
    h_lines = [];
    h_grid = [];
end


d = size(x_nd,2);
params = tps_identity(d);


for iter = 1:opts.n_iter
       
    ypred_nd = tps_eval(x_nd, params);
            
    [xc, yc,w] = find_correspondence(x_nd, ypred_nd, y_md, opts.corr_opts, xfeats_nk, yfeats_mk);
%     h_lines = plot_correspondence(xc, yc, w);    

    if opts.plot_flag, 
       delete(h_lines);    
       delete(h_grid);
       if opts.lines_from_orig, h_lines = plot_correspondence(xc, yc, w); end
       if opts.lines_from_mapped, h_lines = plot_correspondence(tps_eval(xc,params), yc, w); end
       if opts.plot_grid, h_grid = plot_warped_grid(@(x) tps_eval(x, params));
    end


    params = tps_fit(xc, yc, opts.fit_opts);

    if any(isnan(ypred_nd)), error('nans found in ypred'); end
    
    if opts.plot_flag
       delete(h_pred);
%        if d==2
        h_pred = plot(ypred_nd(:,1), ypred_nd(:,2),'gx','MarkerSize',10,'LineWidth',2);
%        end
%        if d==3
%         h_pred = plot3(ypred_nd(:,1), ypred_nd(:,2), ypred_nd(:,3), 'g.');
%        end
           
       if opts.pause_each_iteration, 
           waitforbuttonpress; 
       else
        pause(.05);
       end
    end
    
end



end