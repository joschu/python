from pylab import *
from collections import namedtuple,defaultdict
import scipy.spatial.distance as ssd
import logging
import kinematics.sphere_sampling as ss
from collections import deque
logging.basicConfig(level=logging.INFO,
                    format='%(asctime)s %(name)-12s %(levelname)-8s %(message)s',
                    datefmt='%m-%d %H:%M',
                    filemode='w')

from scipy.spatial import kdtree

logging.getLogger().setLevel(logging.INFO)
class supinfo:
    def __init__(self,inds, supports, best):
        self.inds = inds
        self.supports = supports
        self.best = best
    def __repr__(self):
        return str((self.inds, self.supports, self.best))

#dirs = np.zeros(3,42)
#n_dirs = 42

def decompose(x, thresh):
    
    if x.shape[1] == 2:
        n_dirs = 8
        dirs = np.array([(cos(x), sin(x)) for x in (2*pi*np.arange(n_dirs))/n_dirs])
    elif x.shape[1] == 3:
        dirs = ss.get_sphere_points(1)
        n_dirs = len(dirs)
            
    
        
    
    #dists = ssd.squareform(ssd.pdist(x))
    #pt2nn = dists.argsort(axis=1)[:,:5]
    #def neighbors(ind):
        #return pt2nn[ind]
    
    
    tree = kdtree.KDTree(x)
    def neighbors(ind):
        return tree.query(x[ind], 5)[1]
    
    print "starting"
    n_pts = x.shape[0]

    pt2supports = x.dot(dirs.T)
    pt2label = np.zeros(n_pts, int)
    UNLABELED = -1
    pt2label.fill(UNLABELED)
    
    i_seed = 0
    i_label = 0
    while True:
        
        while True:
            if i_seed == n_pts:
                return pt2label
            if pt2label[i_seed] == UNLABELED:
                break
            i_seed += 1
            
        logging.debug("starting new region with %i", i_seed)
        
        pt2dirs = {}        
        
        dir2supinfo = [supinfo([i_seed], [pt2supports[i_seed][i_dir]], pt2supports[i_seed][i_dir]) for i_dir in xrange(n_dirs)]
        pt2label[i_seed] = i_label
        pt2dirs[i_seed] = set(range(n_dirs))
        frontier = deque()
        rejected = set([])
        
        frontier.extend(neighbors(i_seed))
        print "neighbs", neighbors(i_seed)
        
        while len(frontier) > 0:
            
            
            ######
            #clu = pt2dirs.keys()
            #sup_pd = x[clu,:].dot(dirs.T)
            #best_d = sup_pd.max(axis=0)
            #for i_dir in xrange(n_dirs):
                #i_nearext = [clu[i] for i in np.flatnonzero(best_d[i_dir] - sup_pd[:,i_dir] < thresh)]
                #assert( set(dir2supinfo[i_dir].inds) == set(i_nearext) )
                
                
            
            
            ###            
            
            
            #print dir2supinfo
            #raw_input()

            logging.debug("frontier: %s",frontier)
            i_cur = frontier.popleft()
            if pt2label[i_cur] != UNLABELED or i_cur in rejected: continue
            sup_cur = pt2supports[i_cur]

            logging.debug("checking: %i", i_cur)
            
            print i_cur
            
            reject = False
            
            pt2decrement = defaultdict(int)                        
            for i_dir in xrange(n_dirs):
                cursup = sup_cur[i_dir]
                if cursup > dir2supinfo[i_dir].best:
                    for (i_pt, sup) in zip(dir2supinfo[i_dir].inds, dir2supinfo[i_dir].supports):                        
                        if cursup - sup > thresh:
                            pt2decrement[i_pt] += 1
            for (i_pt, dec) in pt2decrement.items():
                if dec == len(pt2dirs[i_pt]):
                    reject = True
                    logging.debug("rejected because %i would be interior", i_pt)
            if reject: 
                logging.debug("reject")     
                print "reject"
                raw_input()
                rejected.add(i_cur)
            if not reject:
                pt2label[i_cur] = i_label
                pt2dirs[i_cur] = set([i_dir for i_dir in xrange(n_dirs) if dir2supinfo[i_dir].best - sup_cur[i_dir] < thresh])                
                for i_dir in xrange(n_dirs):
                    cursup = sup_cur[i_dir]
                    if cursup > dir2supinfo[i_dir].best:
                        filtinds, filtsups = [],[]
                        for (i_pt, sup) in zip(dir2supinfo[i_dir].inds, dir2supinfo[i_dir].supports):                        
                            if cursup - sup >= thresh:
                                pt2dirs[i_pt].remove(i_dir)
                            else:
                                filtinds.append(i_pt)
                                filtsups.append(sup)
                        filtinds.append(i_cur)
                        filtsups.append(cursup)
                        dir2supinfo[i_dir].inds = filtinds
                        dir2supinfo[i_dir].supports = filtsups
                        dir2supinfo[i_dir].best = cursup
                    elif cursup > dir2supinfo[i_dir].best - thresh:
                        dir2supinfo[i_dir].inds.append(i_cur)
                        dir2supinfo[i_dir].supports.append(cursup)
                logging.debug("adding %i. new clu: %s", i_cur, pt2dirs.keys())
                        
                        
                frontier.extend(neighbors(i_cur))
        i_label += 1

def smalltest():
    ion()
    pts = randn(100,2)
    #pts = np.array([[0,0], [1,0], [0,1], [1,1], [.5, .5]])
    labels = decompose(pts, .05)
    figure(1)
    clf()
    scatter(pts[:,0], pts[:,1],c = labels)
    figure(2)
    for i in xrange(labels.max()+1):
        clf()
        clu = np.flatnonzero(labels == i)
        plot(pts[clu,0], pts[clu,1],'.')
        sup_pd = np.dot(pts[clu,:], dirs.T)
        best_d = sup_pd.max(axis=0)
        print "max deficit",(sup_pd - best_d[None,:]).max(axis=1).min()
        xlim(-2,2)
        ylim(-2,2)
        draw()
        raw_input("press enter")

def bigtest():
    from jds_image_proc.clouds import voxel_downsample
    from jds_image_proc.pcd_io import load_xyz
    import mayavi.mlab as mlab
    
    pts = load_xyz("/home/joschu/Data/scp/three_objs_ds.pcd")
    #pts = voxel_downsample(xyz, .03, False)


    mlab.clf()
    mlab.points3d(pts[:,0], pts[:,1], pts[:,2], color = (1,1,1), scale_factor=.01)
    
    clus = []
    
    labels = decompose(pts, .025)
    for i in xrange(labels.max()+1):
        clu = np.flatnonzero(labels == i)
        clus.append(clu)

    for clu in sorted(clus, key=len, reverse=True):
        if len(clu) < 10: break
        dirs = ss.get_sphere_points(1)
        sup_pd = np.dot(pts[clu,:], dirs.T)
        best_d = sup_pd.max(axis=0)
        print "max deficit",(sup_pd - best_d[None,:]).max(axis=1).min()
        mlab.points3d(pts[clu,0], pts[clu,1], pts[clu,2], color = (rand(),rand(),rand()), scale_factor=.01)
        raw_input()
    

if __name__ == "__main__":
    bigtest()