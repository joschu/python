import comm
import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('rostopic')
import rospy
import rostopic
import numpy as np
from StringIO import StringIO

def getTopicType(rosTopic):
    return rostopic.get_topic_type(rosTopic)[0]
def msg2arr(rosMsg):
    s = StringIO()
    rosMsg.serialize_numpy(s,np)
    return np.fromstring(s.getvalue())
def msgName2Class(name):
    pkg,cls = name.split('/')
    pymodule = pkg+'.msg'
    exec "roslib.load_manifest('%s')"%pkg
    exec "from %s import %s"%(pymodule,cls)
    return eval(cls)


def writerClassFromString(string):
    return eval(string)
def writerClassFromType(rosType):
    name2class = {
        "sensor_msgs/JointState": JointStateWriter,
        "sensor_msgs/Pose": ArrayWriter,
        "sensor_msgs/Image": ImageWriter}        
    return name2class[rosType]


class RosWriter(object):
    extension = "txt"    
    enabled=False
    cbCount = 0

    def __init__(self, topicName, extension, rosTopic, rosMsgName, verbose=False, downsample=1):
        self.rosSub = rospy.Subscriber(rosTopic, msgName2Class(rosMsgName), self.callback)
        self.pub = comm.FilePublisher(topicName, extension)
        self.verbose = verbose
        self.downsample = downsample
        self.count = 0
    def callback(self, rosMsg):
        msg = self.msgFromRos(rosMsg)
        self.cbCount += 1;
        if self.enabled and self.cbCount % self.downsample == 0: 
            self.count += 1
            self.pub.send(msg)
            if self.verbose:
                print "wrote message", self.count
    def msgFromRos(self, rosMsg):
        raise # convert ros message to comm.Message
    def stop(self):
        self.enabled = False
        self.rosSub.unregister()
    def start(self):
        self.enabled = True

class JointStateWriter(RosWriter):
    def msgFromRos(self, rosMsg):
        return comm.ArrayMessage(data=[rosMsg.position,rosMsg.effort])

class ArrayWriter(RosWriter):
    def msgFromRos(self, rosMsg):
        arr = msg2arr(rosMsg)
        return comm.ArrayMessage(data=arr)
    
def rosImage2numpy(image):
    if image.encoding == 'rgb8':
        return np.fromstring(image.data,dtype=np.uint8).reshape(image.height,image.width,3)[:,:,::-1]
    if image.encoding == 'bgr8':
        return np.fromstring(image.data,dtype=np.uint8).reshape(image.height,image.width,3)    
    elif image.encoding == 'mono8':
        return np.fromstring(image.data,dtype=np.uint8).reshape(image.height,image.width)
    elif image.encoding == '32FC1':
        return np.fromstring(image.data,dtype=np.float32).reshape(image.height,image.width)
    else:
        raise Exception    
    
class ImageWriter(RosWriter):
    extension = "jpg"
    def msgFromRos(self, rosMsg):
        arr = rosImage2numpy(rosMsg)
        return comm.MultiChannelImageMessage(data=arr)
    
