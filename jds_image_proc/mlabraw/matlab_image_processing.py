def remove_holes(labels,min_size):
    print "remove holes"
    from scikits.mlabwrap import mlab    
    mlab._session.set("L",labels)
    mlab._session.set("min_size",min_size)
    mlab._session.eval("""
    max_label = max(L(:));
    good_pix = L==0;
    for label = 1:max_label
        good_pix = good_pix | bwareaopen(L==label,min_size);    
    end
    bad_pix = ~logical(good_pix);
    
    [~,I] = bwdist(good_pix,'Chessboard');
    NewL = L;
    NewL(bad_pix) = L(I(bad_pix));
    NewL_d = double(NewL);
    """)
    NewL_d = mlab._session.get("NewL_d")
    return NewL_d.astype('uint8')

def skeletonize(bw):
    from scikits.mlabwrap import mlab
    mlab._session.set("bw",bw.astype('uint8'))
    mlab._session.eval("""
    bw_thin = bwmorph(bw,'thin',Inf);
    bw_thin_d = double(bw);
    """)
    bw_thin_d = mlab._session.get('bw_thin_d')
    return bw_thin_d.astype('uint8')