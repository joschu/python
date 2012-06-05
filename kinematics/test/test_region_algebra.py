import numpy as np
from kinematics import region_algebra as ra

def test_grid2():
    array = np.array([
        [0, 0, 0, 0],
        [0, 1, 1, 0],
        [0, 1, 1, 0],
        [0, 0, 0, 0]])
        
    xticks0 = np.array([3, 4, 5, 6])
    xticks1 = np.array([4.2, 5.2, 6.2, 7.2])
    yticks0 = np.array([0, 1, 2, 3])
    yticks1 = np.array([.8, 1.8, 2.8, 3.8])

    
    grid0 = ra.Grid2(xticks0, yticks0, array, fill_value = 0)
    grid1 = ra.Grid2(xticks1, yticks1, array, fill_value = 0)
    
    array_hopefully = np.array([
        [0, 0, 0, 0, 0],
        [0, 1, 1, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 1, 1, 0],
        [0, 0, 0, 0, 0]])
    # print (grid0 | grid1).array
    
    np.testing.assert_array_almost_equal( (grid0 | grid1).array, array_hopefully)
    
    zero = ra.Grid2.get_additive_identity()
    np.testing.assert_array_almost_equal( (grid0 + zero).array, grid0.array)
    
    one = ra.Grid2.get_multiplicative_identity()
    np.testing.assert_array_almost_equal( (grid0 * one).array, grid0.array)
    
    
if __name__ == "__main__":
    test_grid2()