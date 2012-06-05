clear; close all
load('pointset_pair.mat')

opts = opts_fit;
opts.reg = 0.001;
params = tps_fit(xy1, xy2, opts);

f = @(x) tps_eval(x,params);

xy2est = tps_eval(xy1,params);
clf
hold on
plot(x1, y1,'r')
plot(x2, y2,'g')
plot(xy2est(:,1), xy2est(:,2),'b')


xxyy = axis;
xmin = xxyy(1);
xmax = xxyy(2);
ymin = xxyy(3);
ymax = xxyy(4);

ncoarse = 10;
nfine = 30;

xcoarse = linspace(xmin, xmax, ncoarse)';
ycoarse = linspace(ymin, ymax, ncoarse)';

xfine = linspace(xmin, xmax, nfine)';
yfine = linspace(ymin, ymax, nfine)';

i_handle = 1;
handles = zeros(2*ncoarse,1);

for x = xcoarse'
   xypred = f([x(ones(nfine,1)), yfine]);
   handles(i_handle) = plot(xypred(:,1), xypred(:,2),'c-');
   i_handle = i_handle + 1;
end

for y = ycoarse'
   xypred = f([xfine, y(ones(nfine,1))]);
   handles(i_handle) = plot(xypred(:,1), xypred(:,2),'c-');  
   i_handle = i_handle + 1;
end





[xgrid, ygrid] = meshgrid(xcoarse, ycoarse);
xygrid = [xgrid(:), ygrid(:)];
ngrid = size(xygrid,1);

rots = zeros(ngrid, 2, 2);
for i=1:ngrid
     rots(i,:,:) = eye(2);
end

[xy_result, rot_result] = tps_eval_frames(xygrid, rots, params);

for i=1:ngrid
    pt0 = xy_result(i,:);
    ptx = pt0 + rot_result(i,:,1)*.03;
    pty = pt0 + rot_result(i,:,2)*.03;
    line([pt0(1),ptx(1)],[pt0(2),ptx(2)])
    line([pt0(1),pty(1)],[pt0(2),pty(2)])
end
