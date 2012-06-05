function params = tps_fit_constrained(x_nd, y_nd, xc_kd, plane_kD, opts)
    [n,d] = size(x_nd);
    k = size(xc_kd,1);
    N = n+k;
    
    x_Nd = [x_nd; xc_kd];
    
    dists_NN = squareform(pdist(x_Nd));
    K_NN = tps_kernel(dists_NN, d);
    K_NN(1:N+1:N^2) = 0;
    
    P = homog(x_Nd);
    
    nullP_Nm = null(P');
%     proj_out_neg = null_P * null_P';
    
    
    Kpos_mm = nullP_Nm' * K_NN * nullP_Nm;
%     K_pos = proj_out_neg' * K_NN * proj_out_neg;
%     K_pos = (K_pos + K_pos')/2;
%     K_pos = K_pos + eye(N)*.1;
    m = size(Kpos_mm,1);

    cvx_begin
        variable a_Dd(d+1,d)
        variable w1_m(m,1)
        variable w2_m(m,1)
        
        w1_N = nullP_Nm * w1_m;
        w2_N = nullP_Nm * w2_m;
        
        cost = opts.reg*w1_m' * Kpos_mm * w1_m ...
             + opts.reg*w2_m' * Kpos_mm * w2_m ...
             + sum_square(y_nd(:,1) - homog(x_nd) * a_Dd(:,1) - K_NN(1:n,:) * w1_N) ...
             + sum_square(y_nd(:,2) - homog(x_nd) * a_Dd(:,2) - K_NN(1:n,:) * w2_N);
        
        
        
        minimize( cost )
         subject to
%             w1_N' * P == 0
%             w2_N' * P == 0
              homog(homog(xc_kd) * a_Dd + K_NN(n+1:N,:) * [w1_N,w2_N]) * plane_kD'   <= 0;
    cvx_end

    params.x_nd = x_Nd;
    params.w_nd = nullP_Nm * [w1_m, w2_m];
    params.a_Dd = a_Dd;

end

function x1 = homog(x)
x1 = [x, ones(size(x,1),1)];
end