
    
    
def put3d(handle, name, array):
    mlabraw.put(handle, "flat_array", array.flatten("F"))
    mlabraw.eval(handle, "%s = reshape(flat_array, %i, %i, %i);"%(name, array.shape[0], array.shape[1], array.shape[2]))
def get3d(handle, name):
    mlabraw.eval(handle, """
    flat_array = %s(:);
    shape = size(%s);
    """%(name, name))
    flat_array = mlabraw.get(handle, "flat_array")
    shape = map(int, mlabraw.get(handle, "shape").flat)
    return np.ndarray(buffer = flat_array, shape = shape, order="F")

class NonrigidRegistrationMatlab(object):
    def __init__(self, display = False):
        import mlabraw
        if display:
            self.handle = mlabraw.open("matlab -logfile /tmp/matlablog")       
        else:
            self.handle = mlabraw.open("matlab -nodesktop -nodisplay -nojvm -logfile /tmp/matlablog")       
        mlabraw.eval(self.handle, "addpath('%s');"%os.path.join(os.path.dirname(lfd.__file__), "matlab"))
        
    def fit_transformation_icp(self, points0, points1):
        mlabraw.put(self.handle, "points0", points0)
        mlabraw.put(self.handle, "points1", points1)
        
        mlabraw.eval(self.handle, """
        opts = opts_icp;
        opts.n_iter=6;
        opts.lines_from_orig=1;
        opts.corr_opts.method='bipartite';
        opts.fit_opts.reg = .01;
        %opts.plot_grid = 1;
        opts.corr_opts.bipartite_opts.deficient_col_penalty=1;
        opts.corr_opts.bipartite_opts.deficient_row_penalty=1;
        params = tps_icp(points0, points1, opts);
        save('/tmp/after_fitting.mat');
        """)
        
    def fit_transformation(self, points0, points1):
        assert len(points0) == len(points1)
        mlabraw.put(self.handle, "points0", points0)
        mlabraw.put(self.handle, "points1", points1)
        
        mlabraw.eval(self.handle, """
        opts = opts_fit;
        opts.reg = .01;
        params = tps_fit(points0, points1, opts);
        save('/tmp/after_fitting.mat');
        """)                
        
    def transform_points(self, points):
        
        mlabraw.put(self.handle, "points", points)
        mlabraw.eval(self.handle,"""
        points_result = tps_eval(points, params);        
        """)
        points_result = mlabraw.get(self.handle, "points_result")
        return points_result
        
        
    def transform_poses(self, points, rots):
        mlabraw.put(self.handle, "points", points)                
        put3d(self.handle, "rots", rots)
        
        mlabraw.eval(self.handle,"""
        [points_result, rots_result] = tps_eval_frames(points, rots, params);
        """)
        points_result = mlabraw.get(self.handle,"points_result")
        rots_result = get3d(self.handle, "rots_result")
        
        return points_result, rots_result


        