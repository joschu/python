import mlabraw

MATLAB = None
def initialize():
    global MATLAB
    if MATLAB is None: 
        print "starting matlab..."       
        MATLAB = mlabraw.open("matlab -nodisplay -nosplash -nojvm -nodesktop")
        print "done"
def put(name,array):
    mlabraw.put(MATLAB, name, array)
def get(name):
    return mlabraw.get(MATLAB, name)
def evaluate(string):
    mlabraw.eval(MATLAB, string)

def remove_holes(labels,min_size):
    initialize()
    mlabraw.put(MATLAB, "L",labels)
    mlabraw.put(MATLAB, "min_size",min_size)
    mlabraw.eval(MATLAB, """
    max_label = max(L(:));
    good_pix = L==0;
    for label = 1:max_label
        good_pix = good_pix | bwareaopen(L==label,min_size,4);    
    end
    bad_pix = ~logical(good_pix);
    
    [~,I] = bwdist(good_pix,'Chessboard');
    NewL = L;
    NewL(bad_pix) = L(I(bad_pix));
    NewL_d = double(NewL);
    """)
    NewL_d = mlabraw.get(MATLAB, "NewL_d")
    return NewL_d.astype('uint8')

def branch_points(bw):
    initialize()
    mlabraw.put(MATLAB, "bw",bw)
    mlabraw.eval(MATLAB, """
    bp = bwmorph(bw,'branchpoints')
    bp_d = double(bp);
    """)
    bp_d = mlabraw.get(MATLAB, "bp_d")
    bp =  bp_d.astype('uint8')
    return bp

def remove_branch_points(bw):
    return bw - branch_points(bw)
    

def skeletonize(bw):
    initialize()
    put("bw",bw.astype('uint8'))
    evaluate("""
    bw_thin = bwmorph(bw,'thin',Inf);
    bw_thin_d = double(bw_thin);
    """)
    bw_thin_d = get('bw_thin_d')
    return bw_thin_d.astype('uint8')