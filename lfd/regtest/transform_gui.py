


from traits.api import HasTraits, Instance, Button, on_trait_change, Float,Range
from traitsui.api import View, Item, HSplit, Group, HGroup, VGroup
import openravepy as rave
import numpy as np
from mayavi import mlab
from mayavi.core.ui.api import MlabSceneModel, SceneEditor
    

class CloudAffineTransformer(HasTraits):
    scene = Instance(MlabSceneModel, ())
    
    x00 = Range(-5.,5,1.5)
    x01 = Range(-1.5,1.5,0)
    x02 = Range(-1.5,1.5,0)
    x03 = Range(-1.5,1.5,0)
    x10 = Range(-1.5,1.5,0)
    x11 = Range(-5.,5,1.5)
    x12 = Range(-1.5,1.5,0)
    x13 = Range(-1.5,1.5,0)
    x20 = Range(-1.5,1.5,0)
    x21 = Range(-1.5,1.5,0)
    x22 = Range(-5.,5,1)
    x23 = Range(-1.5,1.5,0)
    
    view = View(

        Item(
          'scene',
          editor=SceneEditor(),
          height=400,
          width=400,
          show_label=False,
        ),
        
        
        VGroup(
        HGroup(Item("x00"), Item("x01"), Item("x02"), Item("x03")),
        HGroup(Item("x10"), Item("x11"), Item("x12"), Item("x13")),
        HGroup(Item("x20"), Item("x21"), Item("x22"), Item("x23"))))
    
    
    def __init__(self, xyz_src, xyz_targ,T=None):
        self.xyz_src = xyz_src
        self.xyz1_src = np.c_[xyz_src, np.ones(len(xyz_src))]
        self.xyz1_targ = np.c_[xyz_targ, np.ones(len(xyz_targ))]
        self.xyz_targ = xyz_targ
        if T is not None:
            Tflat = T.flatten()
            assert Tflat.size == 12 or Tflat.size == 16
            self.x00,self.x01,self.x02,self.x03,self.x10,self.x11,self.x12,self.x13,self.x20,self.x21,self.x22,self.x23 = Tflat[:12]
        
        HasTraits.__init__(self)
        self.redraw()
    def get_t(self):
        return np.array([[self.x00, self.x01, self.x02, self.x03],
                         [self.x10, self.x11, self.x12, self.x13],                         
                         [self.x20, self.x21, self.x22, self.x23],
                         [0,0,0,1]])
    @on_trait_change("anytrait")
    def redraw(self):
        print "redrawing"
        print "["+",   ".join([",".join([str(x) for x in row]) for row in self.get_t()])+"]"
        #print ",".join(str(self.get_t()).split())
        mlab.clf(figure=self.scene.mayavi_scene)
        x,y,z = self.xyz_src.T
        mlab.points3d(x,y,z, color=(1,0,0),scale_factor=.01,figure=self.scene.mayavi_scene)
        x,y,z, = self.xyz1_src.dot(self.get_t().T)[:,:3].T
        mlab.points3d(x,y,z, color=(0,1,0),scale_factor=.01,figure=self.scene.mayavi_scene)
        x,y,z = self.xyz_targ.T
        mlab.points3d(x,y,z, color=(0,0,1),scale_factor=.01,figure=self.scene.mayavi_scene)
