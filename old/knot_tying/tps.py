import jds_utils.conversions as conv
import numpy as np
import scipy.spatial.distance as ssd

def nan2zero(x):
  x[np.isnan(x)] = 0
  return x

def tps_kernel(r):
  return r
  #return nan2zero(r**2 * np.log(r))


class TPS31(object):
  def __init__(self, x_n, y_n, z_n, d_n):
    N = len(x_n)
    dist_nn = ssd.pdist(np.c_[x_n,y_n,z_n])
    k_nn = ssd.squareform(tps_kernel(dist_nn))
    h_n4 = np.c_[np.ones(N), x_n, y_n, z_n]
    L = np.r_[
      np.c_[k_nn,   h_n4],
      np.c_[h_n4.T, np.zeros((4,4))]]
    y = np.r_[d_n, 0, 0, 0, 0]
    wa = np.linalg.solve(L,y)
    self.x_n, self.y_n, self.z_n = x_n, y_n, z_n
    self.w_n, (self.a0, self.ax, self.ay, self.az) = wa[:-4], wa[-4:]
    
  def eval(self, xi_m, yi_m, zi_m):
    dist_mn = ssd.cdist(np.c_[xi_m, yi_m, zi_m], np.c_[self.x_n, self.y_n, self.z_n])
    k_mn = tps_kernel(dist_mn)
    return self.a0 + self.ax*xi_m + self.ay*yi_m + self.az*zi_m + np.dot(k_mn, self.w_n)
  
  def grad(self, xi_m, yi_m, zi_m):
      dist_mn = ssd.cdist(np.c_[xi_m, yi_m, zi_m], np.c_[self.x_n, self.y_n, self.z_n])
      xdiffs_mn = xi_m[:,None] - self.x_n[None,:]
      ydiffs_mn = yi_m[:,None] - self.y_n[None,:]
      zdiffs_mn = zi_m[:,None] - self.z_n[None,:]

      gradx_m = self.ax + np.dot(nan2zero(xdiffs_mn / dist_mn), self.w_n) # df/dx
      grady_m = self.ay + np.dot(nan2zero(ydiffs_mn / dist_mn), self.w_n) 
      gradz_m = self.az + np.dot(nan2zero(zdiffs_mn / dist_mn), self.w_n)        
      
      return np.c_[gradx_m, grady_m, gradz_m]
    
class TPS33(object):
  def __init__(self, x_n, y_n, z_n, xp_n, yp_n, zp_n):
    self.fx = TPS31(x_n, y_n, z_n, xp_n)
    self.fy = TPS31(x_n, y_n, z_n, yp_n)
    self.fz = TPS31(x_n, y_n, z_n, zp_n)
  def eval(self, xi_m, yi_m, zi_m):
    return np.c_[self.fx.eval(xi_m, yi_m, zi_m),self.fy.eval(xi_m, yi_m, zi_m),self.fz.eval(xi_m, yi_m, zi_m)]
  def grad(self, xi_m, yi_m, zi_m):
    # columns of each 3x3 matrix give transformed axes
    # | dx'/dx  dx'/dy  dx'/dz |
    # | dy'/dx  dy'/dy  dy'/dz |
    # | dz'/dx  dz'/dy  dz'/dz |  
    gradx_m3 = self.fx.grad(xi_m, yi_m, zi_m) # [dfx/dx, dfx/dy, dfx, dz] 
    grady_m3 = self.fy.grad(xi_m, yi_m, zi_m)
    gradz_m3 = self.fz.grad(xi_m, yi_m, zi_m)
    
    grad_m33 = np.concatenate([gradx_m3[:,None,:], grady_m3[:,None,:], gradz_m3[:,None,:]], 1)
    
    return grad_m33
  
def transform_poses(pvec_n7, tps33):
  x_n, y_n, z_n = pvec_n7[:,:3].T
  quat_n4 = pvec_n7[:,3:7]
  xp_n, yp_n, zp_n = tps33.eval(x_n, y_n, z_n).T
  grad_n33 = tps33.grad(x_n, y_n, z_n)
  mats_n33 = [conv.quat2mat(quat) for quat in quat_n4]
  tmats_n33 = [np.dot(g,p) for (g,p) in zip(grad_n33, mats_n33)]
  tmats_n33 = [np.linalg.qr(mat)[0] for mat in tmats_n33]
  tquats_n4 = [conv.mat2quat(mat) for mat in tmats_n33]
  
  return np.c_[xp_n, yp_n, zp_n, tquats_n4]