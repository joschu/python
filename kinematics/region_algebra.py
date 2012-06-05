from __future__ import division
import numpy as np


class Grid2:
    """
    2D grid. First axis is X, second axis is Y.
    """
    
    def __init__(self, xticks, yticks, array, fill_value = np.inf):

        self.xticks = np.array(xticks)
        self.yticks = np.asarray(yticks)
        self.array = np.asarray(array)
        self.xres = self.xticks[1] - self.xticks[0]
        self.yres = self.yticks[1] - self.yticks[0]
        
        self.fill_value = fill_value
        
        assert self.xticks.ndim == 1
        assert self.yticks.ndim == 1
        assert self.array.shape == (self.xticks.size, self.yticks.size)
        assert np.allclose(np.diff(self.xticks), self.xres)
        assert np.allclose(np.diff(self.yticks), self.yres)
        
    def __or__(self, other):
        xticks, yticks, self_array, other_array = self.expand_domains(other)
        return Grid2(xticks, yticks, self_array | other_array, fill_value=self.fill_value)
    def __and__(self, other):
        xticks, yticks, self_array, other_array = self.expand_domains(other)
        return Grid2(xticks, yticks, self_array & other_array, fill_value=self.fill_value)
    def __mul__(self, other):
        xticks, yticks, self_array, other_array = self.expand_domains(other)
        return Grid2(xticks, yticks, self_array * other_array, fill_value=self.fill_value)
    def __add__(self, other):
        xticks, yticks, self_array, other_array = self.expand_domains(other)
        return Grid2(xticks, yticks, self_array + other_array, fill_value=self.fill_value)
    def __eq__(self, other):
        return np.allclose(self.xticks, other.xticks) and np.allclose(self.yticks, other.yticks) and np.allclose(self.array, other.array)

    def shift(self, dx, dy):
        return Grid2(self.xticks + dx, self.yticks+dy, self.array, fill_value=self.fill_value)
    def flip(self):
        array = self.array[::-1, ::-1]
        return Grid2(-self.xticks[::-1], -self.yticks[::-1], array, fill_value=self.fill_value)
                
    def expand_domains(self, other):
        """
        Return new arrays that have the same domains, where previously undefined values are set to zero.
        """
        assert np.allclose(self.xres, other.xres)
        assert np.allclose(self.yres, other.yres)
        
        xmin = min(self.xticks[0], other.xticks[0])
        ymin = min(self.yticks[0], other.yticks[0])
        xmax = max(self.xticks[-1], other.xticks[-1])
        ymax = max(self.yticks[-1], other.yticks[-1])
        
        xres = self.xres
        yres = self.yres
        
        xticks = slightly_bigger_arange(xmin, xmax, xres)
        yticks = slightly_bigger_arange(ymin, ymax, yres)
        
        self_array = np.empty((xticks.size, yticks.size), self.array.dtype)
        self_array.fill(self.fill_value)
        other_array = np.empty((xticks.size, yticks.size), other.array.dtype)
        other_array.fill(other.fill_value)
        
        self_xind = int(round( (self.xticks[0] - xticks[0]) / xres))
        self_yind = int(round( (self.yticks[0] - yticks[0]) / yres))
        other_xind = int(round( (other.xticks[0] - xticks[0]) / xres))
        other_yind = int(round( (other.yticks[0] - yticks[0]) / yres))
        
        self_array[self_xind:self_xind+self.xticks.size, self_yind:self_yind+self.yticks.size] = self.array
        other_array[other_xind:other_xind+other.xticks.size, other_yind:other_yind+other.yticks.size] = other.array
        
        return xticks, yticks, self_array, other_array
    
class Grid2AdditiveIdentity(Grid2):
    def __init__(self):
        pass
    def __radd__(self, other):
        return other
    def __add__(self, other):
        return other
        
class Grid2MultiplicativeIdentity(Grid2):
    def __init__(self):
        pass
    def __mul__(self, other):
        return other
    def __rmul__(self, other):
        return other
        
def slightly_bigger_arange(lo, hi, step):
    """
    eg f(0, 2.1, 1) = [0, 1, 2]  but  f(0, 2.8, 1) = [0, 1, 2, 3]
    """
    n_steps = int(round((hi-lo) / step)) + 1
    return lo +  np.arange(n_steps) * step 
