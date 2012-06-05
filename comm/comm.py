import os,json,logging,sys
from os.path import join, exists
import numpy as np
from utils import dir_tools
from glob import glob
from time import time, sleep
try:
    import cv2
except Exception:
    print "didn't import cv2"
from image_proc import pcd_io

DATA_ROOT = "/dont/forget/to/set/data/root"
DATA_FMT_STR = "data%.12i.%s"
INFO_FMT_STR = "info%.12i.json"
LIVE = False
TIMEOUT = 10

def setLive(live):
    global LIVE
    LIVE = live
def setTimeout(timeout):
    global TIMEOUT
    TIMEOUT = timeout
def setDataRoot(path=None):
    global DATA_ROOT
    if path is None:
        DATA_ROOT = os.environ['DATA_ROOT']
    else:
        DATA_ROOT = path
        if not os.path.exists(DATA_ROOT): resetDataDir()
def getDataRoot(): return DATA_ROOT
def resetDataDir():
    dir_tools.mkdir_ask(DATA_ROOT,make_path=True)
    os.mkdir(join(DATA_ROOT,"once"))
def absPath(rel):
    return join(DATA_ROOT,rel)
def topicPath(topic):
    return absPath(topic)
def filePath(basename,topic):
    return absPath(join(topic,basename))
def onceFile(fname):
    return join(DATA_ROOT,"once",fname)
def infoID(infoName):
    return int(infoName[4:16])
def dataID(dataName):
    return int(dataName[4:16])
def waitIfStopped(topic):
    path = filePath("STOP",topic)
    if exists(path): logging.info("%s is throttled. waiting...",path)
    else: return    
    while exists(path):
        sleep(.01)
def setThrottled(topic, newBool):
    throttleFile = filePath("STOP",topic)
    oldBool = exists(throttleFile)
    if newBool and not oldBool: 
        with open(throttleFile,"w"): 
            pass
        print "throttling",topic
    if oldBool and not newBool: 
        os.remove(throttleFile)
        print "unthrottling", topic
def getThrottled(topic):
    throttleFile = filePath("STOP",topic)
    return exists(throttleFile)

def waitIfLive(path):
    if LIVE:
        t_start = time()
        while time() - t_start < TIMEOUT:        
            if os.path.exists(path): return True
            else: sleep(.001)
        print "TIMEOUT!"
        return False
            

    else:
        return os.path.exists(path)
        
def initComm():
    setDataRoot()
    if not exists(join(DATA_ROOT,"once")): os.mkdir(join(DATA_ROOT,"once"))
    if not exists(join(DATA_ROOT,"logs")): os.mkdir(join(DATA_ROOT,"logs"))
    print join(DATA_ROOT,"logs","pipeline_log.txt")
    logging.basicConfig(filename=join(DATA_ROOT,"logs","pipeline_log.txt"),
                        filemode="a",
                        format='%(asctime)s %(message)s', datefmt='%I:%M:%S %p',
                        level=logging.INFO)
    global LIVE,TIMEOUT
    if os.environ.has_key("COMM_LIVE"): 
        LIVE = bool(int(os.environ["COMM_LIVE"]))
    if os.environ.has_key("COMM_TIMEOUT"): 
        TIMEOUT = float(os.environ["COMM_TIMEOUT"])
    
    
class Names(object):
    "basically, an iterator for (dataName, infoName) pairs"
    def __init__(self,topic, extension):
        self.id = 0 # id of next message
        self.extension = extension
        self.topic = topic
    def getCur(self):
        return makeNamePair(self.id,self.extension,self.topic)
    def step(self):
        self.id += 1
    def getCurAndStep(self):
        namePair = self.getCur()
        self.step()
        return namePair
    
def makeNamePair(id,extension,topic):
    return (filePath(DATA_FMT_STR%(id,extension),topic),
            filePath(INFO_FMT_STR%id,topic))
   
#def getExistingPairs(topic):
    #directory = topicPath(topic)
    #dataNames = glob(os.path.join(directory,"data*"))
    #infoNames = glob(os.path.join(directory,"info*"))        
    #if len(dataNames) == len(infoNames): return zip(dataNames,infoNames)
    #elif len(dataNames) == 0: return [(None,infoName) for infoName in infoNames]
    #else: raise Exception("different number of data and info files in topic %s"%topic)

# class TopicWatcher(object):
#     "keep track of how many files are in the topic"
#     def __init__(self,topic):
#         self.directory = topicPath(topic)        

#         namePairs = getExistingPairs(topic)
#         self.counter = len(namePairs)
#         self.notifier = self.makeNotifier()
#     def update(self):        
#         if self.notifier.check_events():
#             self.notifier.read_events()
#             self.notifier.process_events()
#     def makeNotifier(self):
#         wm = pyinotify.WatchManager()
#         wm.add_watch(self.directory,pyinotify.IN_CLOSE_WRITE,rec=False)
#         handler = TopicHandler(self)
#         return pyinotify.Notifier(wm, handler)
#     def __del__(self):
#         self.notifier.stop()

# class TopicHandler(pyinotify.ProcessEvent):    
#     def __init__(self,topicWatcher):
#         self.topicWatcher = topicWatcher
        
#     def process_IN_CLOSE_WRITE(self,event):
#         basename = event.name        
#         if basename.startswith("info"): 
#             assert self.topicWatcher.counter == infoID(basename)
#             self.topicWatcher.counter += 1
                            
class Publisher(object):
    def __init__(self,topic):
        self.topic = topic
    def send(self,message):
        raise
        
class FilePublisher(Publisher):
    def __init__(self,topic,extension):
        Publisher.__init__(self,topic)
        self.names = Names(topic,extension)
        dir_tools.mkdir_ask(topicPath(topic),make_path=True)
    def send(self,message):
        dataName,infoName = self.names.getCurAndStep()  
        message.toFiles(dataName,infoName)     
        logging.info("wrote %s",dataName)
        
        
class FilePutter(object):
    def __init__(self,topic,extension):
        self.topic = topic
        self.extension = extension
        dir_tools.mkdir_ask(topicPath(topic),make_path=True)
    def send_id(self, message, id):
        dataName, infoName = makeNamePair(id, self.extension, self.topic)        
        message.toFiles(dataName,infoName)     
        logging.info("wrote %s",dataName)
        
        
class Subscriber(object):
    def recv(self):
        "get latest message"
        raise
        
class FileSubscriber(Subscriber):
    def __init__(self,topic,extension,msgClass):
        Subscriber.__init__(self)
        self.msgClass = msgClass
        self.names = Names(topic,extension)        
    def recv(self):
        dataName,infoName = self.names.getCurAndStep()
        gotIt = waitIfLive(infoName)
        if gotIt:
            message = self.msgClass()
            message.fromFiles(dataName,infoName)
            return message
        else:
            raise StopIteration
            
class FileGetter(object):
    def __init__(self, topic, extension, msgClass):
        self.topic = topic
        self.extension = extension
        self.msgClass = msgClass
    def recv_id(self, id):
        dataName, infoName = makeNamePair(id, self.extension, self.topic)
        gotIt = waitIfLive(infoName)
        message = self.msgClass()
        message.fromFiles(dataName,infoName)
        return message
        
class LatestFileGetter(object):
    def __init__(self, topic, extension, msgClass):
        self.topic = topic
        self.extension = extension
        self.msgClass = msgClass
    def recv_latest(self):
        infoFile = max(glob(filePath("info*.json",self.topic)))
        id = int(os.path.basename(infoFile)[4:16])        
        dataName, infoName = makeNamePair(id, self.extension, self.topic)
        message = self.msgClass()
        message.fromFiles(dataName,infoName)
        return message

        
class ListSubscriber(Subscriber):
    def __init__(self,msgList):
        self.msgList = msgList
        self.msgIt = iter(self.msgList)
    def recv(self):
        return self.msgIt.next()
class ListPublisher(Publisher):
    def __init__(self):
        self.msgList = []
    def send(self, msg):
        self.msgList.append(msg)

class Message(object):
    "holds data and metadata. can be written to files"
    def __init__(self, data=None, info=None):
        self.data = data
        self.info = info or {}
        if not self.info.has_key("time"): self.info["time"] = time()
    def __cmp__(self,other):
        return -1 if self.data < other.data else 1    
    def writeDataTo(self,dataName):
        pass
    def readDataFrom(self,dataName):
        pass
    def fromFiles(self,dataName,infoName):
        self.data = self.readDataFrom(dataName)
        for i in xrange(10):
            try:
                with open(infoName,"r") as fh: self.info = json.load(fh)
                sleep(.001)
                return
            except ValueError:
                print "failed to read",infoName
        raise ValueError("failed to read json 10 times!")
    def toFiles(self,dataName,infoName):
        self.writeDataTo(dataName)
        with open(infoName,"w") as fh: json.dump(self.info,fh)
    def getTime(self):
        return self.info['time']

class InfoMessage(Message):
    def writeDataTo(self,dataName): pass
    def readDataFrom(self,dataName): pass    
        
class StrMessage(Message):
    def writeDataTo(self,fname):
        with open(fname,"w") as fh: fh.write(self.data)
    def readDataFrom(self,fname):
        with open(fname,"r") as fh: return fh.read()        

class FloatMessage(Message):
    def writeDataTo(self,fname):
        with open(fname,"w") as fh: fh.write(str(self.data))
    def readDataFrom(self,fname):
        with open(fname,"r") as fh: return float(fh.read())
                        
class ArrayMessage(Message):
    def writeDataTo(self,fname):
        np.savetxt(fname,self.data)
    def readDataFrom(self,fname):
        return np.loadtxt(fname)
    
class SingleChannelImageMessage(Message):
    def writeDataTo(self,fname):
        cv2.imwrite(fname, self.data)
    def readDataFrom(self,fname):
        return cv2.imread(fname,0)
    
class MultiChannelImageMessage(Message):
    def writeDataTo(self,fname):
        cv2.imwrite(fname, self.data.copy())
    def readDataFrom(self,fname):
        return cv2.imread(fname)
    
class PointCloudMessage(Message):
    def writeDataTo(self, fname):
        raise NotImplementedError
    def readDataFrom(self,fname):
        return pcd_io.load_xyzrgb(fname)
    
class Retimer(object):
    def __init__(self, sub):
        self.sub = sub
        self.new = True
        self.done = False
    def msgAt(self,time):
        if self.new:
            self.msg0 = self.sub.recv() # msg0: latest message
            self.msg1 = self.sub.recv() # msg1: next-latest message
            self.new = False
        if self.done:
            return self.msg0
        if time < self.msg0.getTime():
            return self.msg0
        while not self.isBetween(time):
            try: self.step()
            except StopIteration: 
                self.done = True
                return self.msg0
        return self.closerMsg(time)
    def isBetween(self,time):
        return time >= self.msg0.getTime() and time <= self.msg1.getTime()
    def step(self):
        self.msg0 = self.msg1
        self.msg1 = self.sub.recv()         
    def closerMsg(self,time):
        diff0 = time - self.msg0.getTime()
        diff1 = self.msg1.getTime() - time
        return self.msg0 if diff0<diff1 else self.msg1

        
class Synchronizer(object):
    def __init__(self, leader, *followers):
        self.leader = leader
        self.followers = followers
        self.retimers = [Retimer(follower) for follower in followers]
    def recvMulti(self):
        leaderMsg = self.leader.recv()
        time = leaderMsg.getTime()
        followerMsgs = [retimer.msgAt(time) for retimer in self.retimers]
        return [leaderMsg] + followerMsgs

        
        
