from pylab import *

def ipf(x):
    for i in xrange(100):
        x0 = x.copy()
        x /= x.sum(axis=1)[:,None]
        x /= x.sum(axis=0)[None,:]
        if i%10==0:
            print abs(x - x0).sum()
    return x

x = rand(4,3)
print ipf(x)