function hs = plot_correspondence(xc, yc, w)

hs = zeros(length(w),1);
d = size(xc,2);

for i=1:length(w)
%    if d==2
       h = line([xc(i,1), yc(i,1)], [xc(i,2), yc(i,2)]);
%    end
%    if d==3
%        h = line([xc(i,1), yc(i,1)], [xc(i,2), yc(i,2)],[xc(i,3), yc(i,3)]);
%    end
       
        
   set(h, 'LineWidth', 3)
   set(h, 'Color', 'y') 
   hs(i) = h;
end

end