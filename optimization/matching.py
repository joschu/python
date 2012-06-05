from __future__ import division
import grb_array as ga
import scipy.spatial.distance as ssd, numpy as np
import gurobipy

class Matcher(object):
    inited = False
    
    def __init__(self):
        self.model = gurobipy.Model()
    
    def match(self, A, B):
        dists = ssd.cdist(A,B)
        if self.inited:
            pass
        else:
            self.ratio = len(B)/len(A)
            self.corr = ga.variable_array(self.model, dists.shape, lb=0)
            ga.add_constraints(self.model, ga.le(self.corr.sum(axis=0) ,1))
            ga.add_constraints(self.model, ga.le(self.corr.sum(axis=1) ,self.ratio*1))
            #ga.add_constraints(self.model, ga.ge(self.corr.sum(axis=0) ,.5))
            #ga.add_constraints(self.model, ga.ge(self.corr.sum(axis=1) ,self.ratio*.5))
            self.cost1 = 10*(self.ratio-self.corr.sum(axis=1)).sum() + 10*(1 - self.corr.sum(axis=0)).sum()

            self.inited = True 
            
        cost0 = gurobipy.LinExpr(dists.reshape(-1).tolist(),self.corr.reshape(-1).tolist())
        self.model.setObjective( cost0 + self.cost1 )                        
        self.model.optimize()
        return ga.get_values(self.corr)
            
            


#@profile
def test_matching():
    from time import time
    import scipy.spatial.distance as ssd
    M = Matcher()
    A = np.random.randn(200,3)
    B = np.random.randn(200,3)
    t_start = time()
    M.match(A,B)
    print time() - t_start, "elapsed"

    t_start = time()
    M.match(A,B)
    print time() - t_start, "elapsed"
    
test_matching()