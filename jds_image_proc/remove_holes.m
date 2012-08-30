function NewL = remove_holes(L,min_size)
max_label = max(L(:));
good_pix = L==0;
for label = 1:max_label
    good_pix = good_pix | bwareaopen(L==label,min_size);    
end
bad_pix = ~logical(good_pix);

[~,I] = bwdist(good_pix,'Chessboard');
NewL = L;
NewL(bad_pix) = L(I(bad_pix));


end