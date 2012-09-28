from sympy import *

a  = Symbol('a')
e = Symbol('jmin')
fx = Symbol('f_x')
fy = Symbol('f_y')
fz = Symbol('f_z')
gx = Symbol('g_x')
gy = Symbol('g_y')
gz = Symbol('g_z')
hx = Symbol('h_x')
hy = Symbol('h_y')
hz = Symbol('h_z')
#fx = Symbol('df/dx')
#fy = Symbol('df/dy')
#fz = Symbol('df/dz')
#gx = Symbol('dg/dx')
#gy = Symbol('dg/dy')
#gz = Symbol('dg/dz')
#hx = Symbol('dh/dx')
#hy = Symbol('dh/dy')
#hz = Symbol('dh/dz')

m = Matrix([[1+a*fx, a*fy, a*fz], \
            [a*gx, 1+a*gy, a*gz], \
            [a*hx, a*hy, 1+a*hz]])

print m
det = m.det()

poly = det - e
print poly

roots = solvers.solve(poly, a)
print len(roots)
print roots
preview(roots[0])
