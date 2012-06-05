function params = tps_fit(x_nd, y_nd, opts, w_n)
    [n,d] = size(x_nd);

    if nargin < 4, w_n = ones(1,n);
    else error('weights not implemented yet')
    end
    
    
    dists_nn = squareform(pdist(x_nd));
    K_nn = tps_kernel(dists_nn, d);
    K_nn(1:n+1:n^2) = 0;
    
    P = [x_nd, ones(n,1)];
    
    
    
    
    
%     remove_P = eye(n) - P / (P' * P) * P';
    
    A = [K_nn + opts.reg*eye(n), P; P', zeros(d+1,d+1)];
    
    
    b = [y_nd; zeros(d+1, d)];
    
%     Aw = bsxfun(@times, A, scaling);
%     Aw = bsxfun(@times, Aw, scaling');
%     bw = bsxfun(@times, b, scaling);
%     
%     coeffs = Aw \ bw;

    coeffs = A \ b;
    params = struct;
    params.x_nd = x_nd;
    params.w_nd = coeffs(1:n,:);
    params.a_Dd = coeffs(n+1:end,:);
% 
% xhom_nD = [x_nd, ones(n,1)];
% 
% cvx_begin
%     variable a_Dd(d+1,d)
%     variable w_nd(n,d)
%     cost = norm(vec(y_nd - xhom_nD * a_Dd - K_nn * w_nd));
%     for i=1:d
%         cost = cost + w_nd(:,i)' * K_nn * w_nd(:,i);
%     end
%     minimize( cost )
% cvx_end
% 
% params.x_nd = x_nd;
% params.w_nd = w_nd;
% params.a_dD = a_Dd;

end
