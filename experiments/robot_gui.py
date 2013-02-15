import traitsui.api as trui
import traits.api as tr
import openravepy as rave
import numpy as np
import trajoptpy
from threading import Thread

class KinBody(tr.HasTraits):

    def __init__(self, robot):
        self.robot = robot
        for joint in robot.GetJoints():
            lower, upper = joint.GetLimits()
            lower = np.clip(lower, -np.pi, np.inf)
            upper = np.clip(upper, -np.inf, np.pi)
            self.add_trait(joint.GetName(), tr.Range(lower[0].item(), upper[0].item(), joint.GetValues()[0]))
        T = robot.GetTransform()
        rx,ry,rz = rave.axisAngleFromRotationMatrix(T[:3,:3])
        tx,ty,tz = T[:3,3]
        self.add_trait("tftx", tr.Range(-5.,5,tx.item()))
        self.add_trait("tfty", tr.Range(-5.,5,ty.item()))
        self.add_trait("tftz", tr.Range(-5.,5,tz.item()))
        self.add_trait("tfrx", tr.Range(-5.,5,rx.item()))
        self.add_trait("tfry", tr.Range(-5.,5,ry.item()))
        self.add_trait("tfrz", tr.Range(-5.,5,rz.item()))
        self.on_trait_change(self.update_robot, 'anytrait')
        tr.HasTraits.__init__(self)
    def update_robot(self, name, new):
        with env:
            joint = self.robot.GetJoint(name)
            if joint:
                self.robot.SetDOFValues([new],[joint.GetDOFIndex()])
            elif name.startswith("tf"):
                T = np.eye(4)
                T[:3,:3] = rave.rotationMatrixFromAxisAngle([self.tfrx, self.tfry, self.tfrz])
                T[:3,3] = [self.tftx, self.tfty, self.tftz]
                self.robot.SetTransform(T)
        print "joints:", self.robot.GetDOFValues()
        print "tf:", self.robot.GetTransform()
        print "active dof", self.robot.GetActiveDOFValues()
    def default_traits_view(self):
        items = []
        for joint in self.robot.GetJoints():
            items.append(trui.Item(name=joint.GetName()))
        items.append(trui.Item("tftx"))
        items.append(trui.Item("tfty"))
        items.append(trui.Item("tftz"))
        items.append(trui.Item("tfrx"))
        items.append(trui.Item("tfry"))
        items.append(trui.Item("tfrz"))
        return trui.View(trui.Group(*items))
        

class Environment(tr.HasTraits):
    def __init__(self, env):
        self.env = env
        bodies = env.GetBodies()
        for body in bodies:
            print "adding ",body.GetName()
            self.add_trait(body.GetName()[:3], KinBody(body))
        tr.HasTraits.__init__(self)

    def default_traits_view(self):
        items = []
        for body in env.GetBodies():
            items.append(trui.Item(name=body.GetName()[:3],style='simple'))
        return trui.View(trui.Group(*items))
    
        
if __name__ == "__main__":
    env = rave.Environment()

    #env.Load('/home/joschu/Proj/trajoptrave/data/pr2-door.env.xml')
    env.Load("data/wamtest1.env.xml")
    #robot.SetActiveDOFs(robot.GetManipulator("rightarm").GetArmJoints(), rave.DOFAffine.X|rave.DOFAffine.Y|rave.DOFAffine.Z|rave.DOFAffine.RotationAxis, [0,0,1])
    #env.Load('/home/joschu/Proj/drc/gfe.xml')
    #env.Load('/home/joschu/Proj/trajoptrave/data/drclogs.env.xml')
    env.StopSimulation()
    viewer = trajoptpy.GetViewer(env)    
    #import ctypes
    #ctypes.pythonapi.PyOS_InputHook
    #cbf = ctypes.CFUNCTYPE(ctypes.c_int)(viewer.Step)    
    from time import sleep
    def plot_loop():
        while True:
            viewer.Step()
            sleep(.03)
    plot_thread = Thread(target=plot_loop)
    plot_thread.start()
    Environment(env).configure_traits()
    plot_thread.join()
