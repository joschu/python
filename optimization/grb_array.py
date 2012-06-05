import gurobipy
from gurobipy import GRB
op = gurobipy.operator
import numpy as np


def variable_array(model, shape, lb=-GRB.INFINITY, ub=GRB.INFINITY, vtype=GRB.CONTINUOUS):
    array = np.zeros(shape, dtype=object)
    for tup in np.ndindex(*shape):
        array[tup] = model.addVar(lb=lb, ub=ub, vtype=vtype)
    model.update()
    return array

def array_map(f, array):
    out = np.zeros(array.shape, dtype=object)
    for (tup, var) in np.ndenumerate(array):
        out[tup] = f(var)
    return out
def array_map_binary(f, array0, array1):
    array0, array1 = np.broadcast_arrays(array0, array1)
    out = np.zeros(array0.shape, dtype=object)
    for (tup, _) in np.ndenumerate(array0):
        out[tup] = f(array0[tup], array1[tup])
    return out
    
def get_values(array):
    out = np.zeros(array.shape)
    for (tup, var) in np.ndenumerate(array):
        out[tup] = array[tup].x
    return out

def add_constraints(model, cons_array):
    for cons in cons_array.flat:
        model.addConstr(cons)

def ge(arr0, arr1):
    return array_map_binary(op.ge, arr0, arr1)
def le(arr0, arr1):
    return array_map_binary(op.le, arr0, arr1)
def l1norm(model, array):
    abs_vals = variable_array(model, array.shape,lb=0)
    add_constraints(model, ge(abs_vals,array))
    return abs_vals.sum()


def test_lstsq():
    from time import time
    
    model = gurobipy.Model()
    A = variable_array(model, (3,4), lb = -1e100, ub=1e100, vtype = GRB.CONTINUOUS)
    B = np.random.randn(4,200)
    C = np.random.randn(3,200)
    
    model.setObjective( l1norm(model, C - np.dot(A,B) ) )
    t_start = time()
    model.optimize()
    print time() - t_start, "elapsed"
    
    
#test_matching()
            
            