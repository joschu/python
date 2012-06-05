function handles = plot_warped_grid(f)

xxyy = axis;
xmin = xxyy(1);
xmax = xxyy(2);
ymin = xxyy(3);
ymax = xxyy(4);

ncoarse = 10;
nfine = 30;

xcoarse = linspace(xmin, xmax, ncoarse);
ycoarse = linspace(ymin, ymax, ncoarse);

xfine = linspace(xmin, xmax, nfine)';
yfine = linspace(ymin, ymax, nfine)';

i_handle = 1;
handles = zeros(2*ncoarse,1);

for x = xcoarse
   xypred = f([x(ones(nfine,1)), yfine]);
   handles(i_handle) = plot(xypred(:,1), xypred(:,2),'c-');
   i_handle = i_handle + 1;
end

for y = ycoarse
   xypred = f([xfine, y(ones(nfine,1))]);
   handles(i_handle) = plot(xypred(:,1), xypred(:,2),'c-');  
   i_handle = i_handle + 1;
end


end