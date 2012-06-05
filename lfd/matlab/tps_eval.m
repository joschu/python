function ypred_nd = tps_eval(x_md, params)

[m,d] = size(x_md);

dist_mn = pdist2(x_md, params.x_nd);
K_mn = tps_kernel(dist_mn,d);

xhom_mD = [x_md, ones(m,1)];
ypred_nd = K_mn * params.w_nd + xhom_mD * params.a_Dd;

end