function [newx,newy,w] = find_correspondence(x_md, mapx_md, y_nd, opts, xfeats_mk, yfeats_nk)

if nargin <= 4
    xfeats_mk = [];
    yfeats_nk = [];
end

newxc = {};
newyc = {};
w = [];

switch opts.method
    case 'nn'
        nncorr_mn = calc_match_matrix_nn([mapx_md, xfeats_mk], [y_nd,yfeats_nk], opts);
    case 'bipartite'
        if ~isfield(opts,'bipartite_opts'), opts.bipartite_opts = struct(); end
        nncorr_mn = calc_match_matrix_bipartite([mapx_md,xfeats_mk], [y_nd,yfeats_nk], opts.bipartite_opts);
    otherwise
        error('method must be nn or bipartite')
end

for i_x = 1:size(x_md, 1)
    yinds = find(nncorr_mn(i_x,:)>.1);
    if ~isempty(yinds)
        meany = mean(y_nd(yinds,:),1);
        newxc{end+1,1} = x_md(i_x, :);
        newyc{end+1,1} = meany;
        w(end+1,1) = length(yinds);
%         for yind = yinds
%            newxc{end+1,1} = x_md(i_x,:);
%            newyc{end+1,1} = y_nd(yind,:);
%            w(end+1,1) = nncorr_mn(i_x, yind;
%         end
    end
end

newx = cell2mat(newxc);
newy = cell2mat(newyc);
        
        
        
        


end

function nncorr_mn = calc_match_matrix_bipartite(x_mn, y_nd, opts)
nncorr_mn = match_with_cvx(x_mn, y_nd, opts);

end

function nncorr_mn = calc_match_matrix_nn(x_md, y_nd, opts)
m = size(x_md,1);
n = size(y_nd,1);

dists_mn = pdist2(x_md, y_nd);
[nndist_m, nnind_m] = min(dists_mn,[],2);
[nndist_n, nnind_n] = min(dists_mn,[],1);

nncorr_mn = zeros(m,n);
for i_x = 1:m
    if nndist_m(i_x) < opts.max_dist
        i_y = nnind_m(i_x);
        nncorr_mn(i_x, i_y) = nncorr_mn(i_x, i_y) + 1;
    end
end

for i_y = 1:n
    if nndist_n(i_y) < opts.max_dist
        i_x = nnind_n(i_y);
        nncorr_mn(i_x, i_y) = nncorr_mn(i_x, i_y) + 1;
    end
end
end
