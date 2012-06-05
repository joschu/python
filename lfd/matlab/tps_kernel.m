function K = tps_kernel(dist, dim)
switch dim
    case 1
        K = dist.^3;
    case 2
        K = dist.^2 .* log(dist);
        K(dist == 0) = 0;
    case 3
        K = -dist;        
end
end