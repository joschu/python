from ransac import ransac, ConstantModel
import numpy as np
from utils import testing

@testing.testme
def test_ConstantModel():
    true_inliers = np.random.rand(10,3)
    true_outliers = np.zeros((3,3)) + 100
    data = np.concatenate([true_inliers, true_outliers])
    true_inlier_inds = np.arange(10,dtype=int)
    
    est_model, est_inlier_inds, est_error = ransac(data, ConstantModel, 1, 10, 1, 5)
    assert np.all(est_inlier_inds == true_inlier_inds)


if __name__ == "__main__":
    testing.test_all(stop=True)
