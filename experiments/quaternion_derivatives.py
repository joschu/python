from sympy import *
import re
def quat_mult(q1, q2):
  return (
    q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
    q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
    q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3],
    q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1],
)
def quat_conj(q):
  return (q[0], -q[1], -q[2], -q[3])
    
    
qw, qx, qy, qz, x, y, z = var("qw qx qy qz x y z")
q = (qw, qx, qy, qz)
rotated_xyz = quat_mult(q, quat_mult((0, x, y, z), quat_conj(q)))[1:]


jac = zeros((3,4))


def format_matrix(m):
  out = "{"
  for row in xrange(m.shape[0]):
    for col in xrange(m.shape[1]-1):
      out += str(m[row, col])
      out += ", "
    out += str(m[row, m.shape[1]-1])
    if (row < m.shape[0]-1): out += ",\n"
    
  out += "}"
  return re.sub(r"(\w+)\*\*2",r"\1*\1", out)
  
  return out


for o in xrange(3):
  for i in xrange(4):
    jac[o,i] = diff(rotated_xyz[o], q[i])

print format_matrix(jac)

#proj = eye(4) - Matrix([qw, qx, qy, qz]) * Matrix([qw, qx, qy, qz]).T
#print format_matrix(jac * proj)