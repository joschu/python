import scipy.optimize as so
import numpy as np
import rospy
import time
import scipy.spatial.distance as ssd
from jds_utils.colorize import colorize



def nan2zero(x):
    np.putmask(x, np.isnan(x), 0)
    return x

def tps_eval(x_ma, lin_ag, trans_g, w_ng, x_na):
    K_mn = ssd.cdist(x_ma, x_na)
    return np.dot(K_mn, w_ng) + np.dot(x_ma, lin_ag) + trans_g[None,:]

def tps_grad(x_ma, lin_ag, trans_g, w_ng, x_na):
    N, D = x_na.shape
    M = x_ma.shape[0]

    dist_mn = ssd.cdist(x_ma, x_na,'euclidean')

    grad_mga = np.empty((M,D,D))

    lin_ga = lin_ag.T
    for a in xrange(D):
        diffa_mn = x_ma[:,a][:,None] - x_na[:,a][None,:]
        grad_mga[:,:,a] = lin_ga[None,:,a] + np.dot(nan2zero(diffa_mn/dist_mn),w_ng)
    return grad_mga
    
def tps_nonrigidity_grad(x_ma, lin_ag, trans_g, w_ng, x_na, return_tuple = False):
    N, D = x_na.shape
    M = x_ma.shape[0]

    dists_mn = ssd.cdist(x_ma, x_na,'euclidean')
    diffs_mna = x_ma[:,None,:] - x_na[None,:,:]

    grad_mga = np.empty((M,D,D))
    lin_ga = lin_ag.T
    for a in xrange(D):
        grad_mga[:,:,a] = lin_ga[None,:,a] + np.dot(nan2zero(diffs_mna[:,:,a]/dists_mn),w_ng)

    # m n g a b
    Jw_mngab = (nan2zero(diffs_mna[:,:,None,:,None]/dists_mn[:,:,None,None,None])) * grad_mga[:,None,:,None,:]
    Jw_mngab = Jw_mngab + Jw_mngab.transpose(0,1,2,4,3)        
    Jw_mabng = Jw_mngab.transpose(0,3,4,1,2)
    Jw = Jw_mabng.reshape(M*D**2,N*D)
    
    Jl_mcgab = np.eye(D)[None,:,None,:,None]*grad_mga[:,None,:,None,:]
    Jl_mcgab = Jl_mcgab + Jl_mcgab.transpose(0,1,2,4,3)
    Jl_mabcg = Jl_mcgab.transpose(0,3,4,1,2)
    Jl = Jl_mabcg.reshape(M*D**2,D*D)
    
    Jt = np.zeros((M*D**2,D))
    
    if return_tuple:
        return Jl, Jt, Jw
    else:
        J = np.c_[Jl, Jt, Jw]
        return J
    
    
    
def tps_nonrigidity(x_ma, lin_ag, trans_g, w_ng, x_na):
    M,D = x_ma.shape

    grad_mga = tps_grad(x_ma, lin_ag, trans_g, w_ng, x_na)
    err_mab = np.empty((M,D,D))
    for m in xrange(M):
        err_mab[m] = np.dot(grad_mga[m].T, grad_mga[m]) - np.eye(D)
    return err_mab.flatten()


def tps_nr_eval(lin_ag, trans_g, w_ng, x_na, y_ng, xnr_ma, bend_coef, nr_coef, K_nn = None, return_tuple=False):
    D = lin_ag.shape[0]
    K_nn = K_nn or ssd.squareform(ssd.pdist(x_na))
    ypred_ng = np.dot(K_nn, w_ng) + np.dot(x_na, lin_ag) + trans_g[None,:]
    res_cost = ((ypred_ng - y_ng)**2).sum()
    bend_cost = bend_coef * sum(np.dot(w_ng[:,g], np.dot(-K_nn, w_ng[:,g])) for g in xrange(D))
    nr_cost = nr_coef * (tps_nonrigidity(xnr_ma, lin_ag, trans_g, w_ng, x_na)**2).sum()
    if return_tuple:
        return res_cost, bend_cost, nr_cost, res_cost + bend_cost + nr_cost
    else:
        return res_cost + bend_cost + nr_cost

def tps_fit(x_na, y_ng, bend_coef, rot_coef, wt_n=None):
    N,D = x_na.shape
        
    K_nn = ssd.squareform(ssd.pdist(x_na))
    coef_ratio = bend_coef / rot_coef if rot_coef > 0 else 0
    if wt_n is None: reg_nn = bend_coef * np.eye(N)
    else: reg_nn = bend_coef/(wt_n + 1e-6)
    
    
    A = np.r_[
        np.c_[K_nn - reg_nn,   x_na,    np.ones((N,1))],
        np.c_[x_na.T,      coef_ratio * np.eye(D), np.zeros((D,1))],
        np.c_[np.ones((1,N)),     np.zeros((1,D)), 0]]
    B = np.r_[
        y_ng,
        coef_ratio * np.eye(D),
        np.zeros((1, D))
    ]
    
    X = np.linalg.solve(A, B)
    w_ng = X[:N,:]
    lin_ag = X[N:N+D,:]
    trans_g = X[N+D,:]
    return lin_ag, trans_g, w_ng
    

def tps_fit2(x_na, y_ng, bend_coef, rot_coef):
    N,D = x_na.shape
    _u,_s,_vh = np.linalg.svd(np.c_[x_na, np.ones((N,1))])
    N_nq = _u[:,4:] # null of data
    K_nn = ssd.squareform(ssd.pdist(x_na))
    Q_nn = np.c_[x_na, np.ones((N,1)),K_nn.dot(N_nq)]
    QQ_nn = np.dot(Q_nn.T, Q_nn)
    
    A = QQ_nn
    A[4:, 4:] -= bend_coef * N_nq.T.dot(K_nn).dot(N_nq)
    B = Q_nn.T.dot(y_ng)

    A[:3, :3] += rot_coef * np.eye(3)
    B[:3, :3] += rot_coef * np.eye(3)
    
    X = np.linalg.solve(A,B)

    lin_ag = X[:D,:]
    trans_g = X[D,:]    
    w_ng = N_nq.dot(X[D+1:,:])
    return lin_ag, trans_g, w_ng

def tps_nr_fit(x_na, y_ng, bend_coef, nr_ma, nr_coef, method="newton"):
    N,D = x_na.shape
    lin_ag, trans_g, w_ng = tps_fit2(x_na, y_ng, bend_coef, 1e-3)
    #return lin_ag, trans_g, w_ng

    ##for testing that it takes one step when nonrigidity cost is zero:
    #lin_ag, trans_g, w_ng = tps_fit(x_na, y_ng, bend_coef, 0)
    #res_cost, bend_cost, nr_cost, fval = tps_nr_eval(lin_ag, trans_g, w_ng, x_na, nr_ma, bend_coef, nr_coef, return_tuple=True)
    #print "CORRECT COST, res,bend,nr,total = %.3e, %.3e, %.3e, %.3e"%(res_cost, bend_cost, nr_cost, fval)
    #lin_ag += np.random.randn(*lin_ag.shape)*5
    #res_cost, bend_cost, nr_cost, fval = tps_nr_eval(lin_ag, trans_g, w_ng, x_na, nr_ma, bend_coef, nr_coef, return_tuple=True)
    #print "NOISE ADDED COST, res,bend,nr,total = %.ef, %.3e, %.3e, %.3e"%(res_cost, bend_cost, nr_cost, fval)
    
    _u,_s,_vh = np.linalg.svd(np.c_[x_na, np.ones((N,1))])
    N_nq = _u[:,4:] # null of data
    #w_ng = N_nq.dot(N_nq.T.dot(w_ng))
        
    K_nn = ssd.squareform(ssd.pdist(x_na))
    Q_nn = np.c_[x_na, np.ones((N,1)),K_nn.dot(N_nq)]
    QQ_nn = np.dot(Q_nn.T, Q_nn)
    Bend_nn = np.zeros((N,N))
    Bend_nn[4:, 4:] = - N_nq.T.dot(K_nn.dot(N_nq))
    
    n_iter=1000
    for i in xrange(n_iter):
        X_ng = np.r_[lin_ag, trans_g[None,:], N_nq.T.dot(w_ng)]

        res_cost, bend_cost, nr_cost, fval = tps_nr_eval(lin_ag, trans_g, w_ng, x_na, y_ng, nr_ma, bend_coef, nr_coef, return_tuple=True)
        print colorize("iteration %i, cost %.3e"%(i, fval), 'red'),
        print "= %.3e (res) + %.3e (bend) + %.3e (nr)"%(res_cost, bend_cost, nr_cost)
                
        
        Jl_zcg, Jt_zg, Jw_zng = tps_nonrigidity_grad(nr_ma, lin_ag, trans_g, w_ng, x_na, return_tuple=True)
        nrerr_z = tps_nonrigidity(nr_ma, lin_ag, trans_g, w_ng, x_na)
        
        
        if method == "newton":
            fullstep_ng = np.empty((N,D))
            for g in xrange(D):
                J_zn = np.c_[Jl_zcg[:,g::D], Jt_zg[:,g::D], Jw_zng[:,g::D].dot(N_nq)]
                JJ_nn = np.dot(J_zn.T, J_zn)
                A = nr_coef*JJ_nn + QQ_nn + bend_coef*Bend_nn
                X0 = X_ng[:,g]
                B = nr_coef*np.dot(J_zn.T, np.dot(J_zn, X0) - nrerr_z) + Q_nn.T.dot(y_ng[:,g])
                fullstep_ng[:,g] = np.linalg.solve(A,B) - X_ng[:,g]

        elif method == "gradient":
            def eval_partial(cand_X_ng):
                cand_X_ng = cand_X_ng.reshape(-1,3)
                cand_lin_ag, cand_trans_g, cand_w_ng = cand_X_ng[:D], cand_X_ng[D], N_nq.dot(cand_X_ng[D+1:])
                fval_cand = tps_nr_eval(cand_lin_ag, cand_trans_g, cand_w_ng, x_na, y_ng, nr_ma, bend_coef, nr_coef)
                return fval_cand
            def eval_partial2(cand_X_ng):
                return ((Q_nn.dot(X_ng) - y_ng)**2).sum()
            def eval_partial3(cand_X_ng):
                cand_X_ng = cand_X_ng.reshape(-1,3)
                cand_lin_ag, cand_trans_g, cand_w_ng = cand_X_ng[:D], cand_X_ng[D], N_nq.dot(cand_X_ng[D+1:])
                return ((y_ng - tps_eval(x_na, cand_lin_ag, cand_trans_g, cand_w_ng, x_na))**2).sum()
            
            
            grad_ng = np.empty((N,D))
            for g in xrange(D-1,-1,-1):
                Jnr_zn = np.c_[Jl_zcg[:,g::D], Jt_zg[:,g::D], Jw_zng[:,g::D].dot(N_nq)]
                grad_ng[:,g] = 2 * nr_coef * nrerr_z.dot(Jnr_zn) \
                    + 2 * Q_nn.T.dot(Q_nn.dot(X_ng[:,g]) - y_ng[:,g]) \
                    + 2 * bend_coef * Bend_nn.dot(X_ng[:,g])

            #assert np.allclose(eval_partial2(X_ng), eval_partial3(X_ng))
            #assert np.allclose(eval_partial(X_ng), eval_partial2(X_ng))
            #grad0_ng = ndt.Gradient(eval_partial)(X_ng.flatten()).reshape(-1,3)
            fullstep_ng = -grad_ng
            #assert np.allclose(grad0_ng, grad_ng)
            
            
            

        cost_improved = False
        for stepsize in 3.**np.arange(0,-10,-1):
            cand_X_ng = X_ng + fullstep_ng*stepsize
            cand_lin_ag, cand_trans_g, cand_w_ng = cand_X_ng[:D], cand_X_ng[D], N_nq.dot(cand_X_ng[D+1:])
            fval_cand = tps_nr_eval(cand_lin_ag, cand_trans_g, cand_w_ng, x_na, y_ng, nr_ma, bend_coef, nr_coef)
            print "stepsize: %.1g, fval: %.3e"%(stepsize, fval_cand)
            if fval_cand < fval:
                cost_improved = True
                break
        if not cost_improved:
            print "couldn't improve objective"
            break

            
        lin_ag = cand_lin_ag
        trans_g = cand_trans_g
        w_ng = cand_w_ng
    return lin_ag, trans_g, w_ng









def test_nr_grad():
    import numdifftools as ndt    
    D = 3
    pts0 = np.random.randn(10,D)
    pts_eval = np.random.randn(3,D)
    def err_partial(params):
        lin_ag = params[0:9].reshape(3,3)
        trans_g = params[9:12]
        w_ng = params[12:].reshape(-1,3)
        return tps_nonrigidity(pts_eval, lin_ag, trans_g, w_ng, pts0)    
    lin_ag, trans_g, w_ng = np.random.randn(D,D), np.random.randn(D), np.random.randn(len(pts0), D)
    J1 = tps_nonrigidity_grad(pts_eval, lin_ag, trans_g, w_ng, pts0)
    J = ndt.Jacobian(err_partial)(np.r_[lin_ag.flatten(), trans_g.flatten(), w_ng.flatten()])
    assert np.allclose(J1, J)
def test_jacobian():
    import numdifftools as ndt
    D = 3
    pts0 = np.random.randn(100,3)
    pts_eval = np.random.randn(20,D)
    lin_ag, trans_g, w_ng, x_na = np.random.randn(D,D), np.random.randn(D), np.random.randn(len(pts0), D), pts0    
    def eval_partial(x_ma_flat):
        x_ma = x_ma_flat.reshape(-1,3)
        return tps_eval(x_ma, lin_ag, trans_g, w_ng, pts0)
    for i in xrange(len(pts_eval)):
        rots = ndt.Jacobian(eval_partial)(pts_eval[i])
        rots1 = tps_grad(pts_eval[i:i+1], lin_ag, trans_g, w_ng, pts0)
        assert np.allclose(rots1, rots)

def test_tps_fit_equiv():
    pts0 = np.random.randn(100,3)
    pts1 = np.random.randn(100,3)
    lin_ag, trans_g, w_ng = tps_fit(pts0, pts1, .01, 0)    
    lin2_ag, trans2_g, w2_ng = tps_fit2(pts0, pts1, .01, 0)
    
    assert np.allclose(lin_ag, lin2_ag)
    assert np.allclose(trans_g, trans2_g)
    assert np.allclose(w_ng, w2_ng)

def test_fit_nr():
    from jds_image_proc.clouds import voxel_downsample
    from brett2.ros_utils import RvizWrapper    
    import lfd.registration as lr
    import lfd.warping as lw    
    if rospy.get_name() == "/unnamed":
        rospy.init_node("test_rigidity", disable_signals=True)
        from time import sleep
        sleep(1)
    rviz = RvizWrapper.create()
    
    pts0 = np.loadtxt("test/rope_control_points.txt")
    pts1 = np.loadtxt("test/bad_rope.txt")    
    pts_rigid = voxel_downsample(pts0, .02)
    lr.Globals.setup()
    np.seterr(all='ignore')
    np.set_printoptions(suppress=True)

    pts_rigid = pts0
    lin_ag, trans_g, w_ng = tps_nr_fit(pts0, pts1, 0.01, pts0, .1, method="newton")
    #lin_ag2, trans_g2, w_ng2 = tps_fit(pts0, pts1, .01, .01)
    #assert np.allclose(w_ng, w_ng2)
    def eval_partial(x_ma):
        return tps_eval(x_ma, lin_ag, trans_g, w_ng, pts0) 
    lr.plot_orig_and_warped_clouds(eval_partial, pts0, pts1, res=.008)
    #handles = lw.draw_grid(rviz, eval_partial, pts0.min(axis=0), pts0.max(axis=0), 'base_footprint')

    grads = tps_grad(pts_rigid, lin_ag, trans_g, w_ng, pts0)
    print "worst violation:",np.max(np.abs([grad.T.dot(grad)-np.eye(3) for grad in grads]))


if __name__ == "__main__":
    np.seterr(all='ignore')
    print "test_jacobian"
    test_jacobian()
    print "test_tps_fit_equiv"
    test_tps_fit_equiv()
    print "test_nr_grad"
    test_nr_grad()
    #test_fit_nr()    