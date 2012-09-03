import scipy.interpolate as interp
import numpy as np

def _to_spline(curve):
  return interp.splprep(curve.transpose(), k=5, s=0.05)

def _eval_spline(spline):
  return np.asarray(interp.splev(spline[1], spline[0])).transpose()

def _get_spline_control_pts(spline):
  return np.asarray(spline[0][1]).transpose()

def _perturb_control_pts(in_pts, s=0.01):
  '''in_pts: Nx3 array
     s: variance'''
  ret = np.copy(in_pts)
  pts = ret[:,0:2] # only perturb in the x-y plane
  cov = [[s, 0], [0, s]]
  for i in range(len(pts)):
    pts[i] = np.random.multivariate_normal(pts[i], cov)
  return ret

def _perturb_spline(spline, s=0.01):
  new_ctl_pts = _perturb_control_pts(_get_spline_control_pts(spline), s)
  return ([spline[0][0], new_ctl_pts.transpose(), spline[0][2]], spline[1])

def _curvelen(c):
  l = 0
  for i in range(len(c)-1): l += np.linalg.norm(c[i] - c[i+1])
  return l

def _adjust_curve(curve, target_curve, tolerance=0.01, maxiter=100):
  '''using binary search, shrinks curve to the same length as target_curve
     and translates to match centroids'''
  center, target_center = np.mean(curve, axis=0), np.mean(target_curve, axis=0)
  curve2 = curve - center
  tclen = _curvelen(target_curve)
  slo, shi = 0, 100
  for _ in range(maxiter):
    smid = (shi+slo)/2.
    adj_curve2 = curve2*smid
    adj_clen = _curvelen(adj_curve2)
    if adj_clen > tclen and adj_clen - tclen > tolerance: shi = smid
    elif adj_clen < tclen and tclen - adj_clen > tolerance: slo = smid
    else: break
  return curve2*smid + target_center

def _resample_spline(spline, tolerance=0.0001):
  '''returns a new spline with u-values adjusted so that the evaluated curve's points are spaced equally'''
  def udist(spline, u0, u1):
    c = np.asarray(interp.splev([u0, u1], spline[0])).transpose()
    return np.linalg.norm(c[0] - c[1])
  u = spline[1]
  target_ulen = _curvelen(_eval_spline(spline)) / float(len(u)-1)
  newu = []
  for i in range(len(u)):
    if i == 0:
      newu.append(u[0])
      continue
    # adjust u[i] until udist(u[i-1], u[i]) == target_ulen
    ulo, uhi = newu[i-1]-0.5, newu[i-1]+0.5
    for _ in range(100):
      umid = (uhi+ulo)/2.
      ulen = udist(spline, newu[i-1], umid)
      if ulen > target_ulen and ulen - target_ulen > tolerance: uhi = umid
      elif ulen < target_ulen and target_ulen - ulen > tolerance: ulo = umid
      else: break
    newu.append(umid)
  assert len(u) == len(newu)
  return ([spline[0][0], spline[0][1], spline[0][2]], newu)

def perturb_curve(curve, s=0.01):
  new_spline = _resample_spline(_perturb_spline(_to_spline(curve), s))
  return _adjust_curve(_eval_spline(new_spline), curve)

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('--s', action='store', type=float, default=0.01, help='variance of randomness to introduce to the b-spline control points')
  parser.add_argument('--n', action='store', type=int, default=1, help='num samples to draw')
  parser.add_argument('input')
  parser.add_argument('output')
  args = parser.parse_args()

  import os
  assert os.path.exists(args.input)
  assert os.path.exists(os.path.dirname(args.output))

  import matplotlib.pyplot as plt
  curve = np.loadtxt(args.input)
  in_plot = plt.plot(curve[:,0], curve[:,1], 'b.-')

  spline = _to_spline(curve)

  #curve2 = _eval_spline(spline)
  #out_plot = plt.plot(curve2[:,0], curve2[:,1], 'r.-')

  #ctl_pts = _get_spline_control_pts(spline)
  #plt.plot(ctl_pts[:,0], ctl_pts[:,1], 'yo')

  for _ in range(args.n):
    new_curve = perturb_curve(curve, args.s)
    plt.plot(new_curve[:,0], new_curve[:,1], 'm.-')

  #new_ctl_pts = _get_spline_control_pts(new_spline)
  #plt.plot(new_ctl_pts[:,0], new_ctl_pts[:,1], 'go')

  plt.axis('equal')
  plt.show()
