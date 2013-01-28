from __future__ import division
from pylab import *

def f(omega, t):
    return np.abs( (1-t)*1 + t*np.exp(omega*1j) - np.exp(omega*1j*t) )

ts = linspace(0,1,1001)
for omega in linspace(0,pi,10000):
    fvals = f(omega, ts)
    imax = fvals.argmax()
    tmax = ts[imax]
    mybound = omega**2/2 * .25
    if omega > 0: 
        assert tmax == .5
        assert fvals[imax] <= mybound