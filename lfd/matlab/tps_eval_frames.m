function [x_warped_md, rot_warped_mdd] = tps_eval_frames(x_md, rot_mdd, params)


[m,d] = size(x_md);
n = size(params.x_nd, 2);

dist_mn = pdist2(x_md, params.x_nd);
K_mn = tps_kernel(dist_mn, d);

grad_mdd = zeros(m,d,d);

if d==3
    xdiffs_mn = bsxfun(@minus, x_md(:,1), params.x_nd(:,1)');
    ydiffs_mn = bsxfun(@minus, x_md(:,2), params.x_nd(:,2)');
    zdiffs_mn = bsxfun(@minus, x_md(:,3), params.x_nd(:,3)');
    
    grad_mdd(:,:,1) = bsxfun(@plus, params.a_Dd(1,:), ...
        invalid2zero(xdiffs_mn ./ dist_mn) * params.w_nd(:,1));
    grad_mdd(:,:,2) = bsxfun(@plus, params.a_Dd(2,:), ...
        invalid2zero(ydiffs_mn ./ dist_mn) * params.w_nd(:,2));
    grad_mdd(:,:,3) = bsxfun(@plus, params.a_Dd(3,:), ...
        invalid2zero(zdiffs_mn ./ dist_mn) * params.w_nd(:,3));               
elseif d==2
    xdiffs_mn = bsxfun(@minus, x_md(:,1), params.x_nd(:,1)');
    ydiffs_mn = bsxfun(@minus, x_md(:,2), params.x_nd(:,2)');
    
    grad_mdd(:,:,1) = bsxfun(@plus, params.a_Dd(1,:), ...
        invalid2zero(xdiffs_mn .* (2*log(dist_mn)+1)) * params.w_nd);
    grad_mdd(:,:,2) = bsxfun(@plus, params.a_Dd(2,:), ...
        invalid2zero(ydiffs_mn .* (2*log(dist_mn)+1)) * params.w_nd);
%     grad_mdd(:,:,1) =  invalid2zero(xdiffs_mn .* (2*log(dist_mn)+1)) * params.w_nd;
%     grad_mdd(:,:,2) =  invalid2zero(ydiffs_mn .* (2*log(dist_mn)+1)) * params.w_nd;
    
end



rot_warped_mdd = zeros(m,d,d);
for i_m = 1:m
   rot_warped_mdd(i_m,:,:) =  squeeze(grad_mdd(i_m,:,:)) * squeeze(rot_mdd(i_m,:,:));
end


xhom_mD = [x_md, ones(m,1)];
x_warped_md = K_mn * params.w_nd + xhom_mD * params.a_Dd;

end

function x=invalid2zero(x)
x(~isfinite(x))=0;
end