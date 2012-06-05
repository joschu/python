from comm import *
import comm
import utils.dir_tools
import shutil,os
from os.path import join,basename
from utils.testing import testme, test_all
import threading

setDataRoot("/home/joschu/Data/test_comm")
dir_tools.ensure_exists(getDataRoot())

def delTopic(topic):
    path = topicPath(topic)
    if os.path.exists(path):
        shutil.rmtree(path)
def touch(path):
    with open(path,'w') as fh: pass
        
@testme        
def test_Names():
    topic = "test_Names"
    delTopic(topic)
    names = Names("test_Names","txt")
    dataName, infoName = names.getCur()
    assert basename(dataName) == "data000000000000.txt"
    assert basename(infoName) == "info000000000000.json"
    names.step()
    dataName, infoName = names.getCur()
    assert basename(dataName) == "data000000000001.txt"
    assert basename(infoName) == "info000000000001.json"
    
# @testme    
# def test_TopicWatcher():
#     topic = "test_TopicWatcher"
#     delTopic(topic)
#     pub = FilePublisher(topic,"txt")    
#     topicWatcher = TopicWatcher(topic)    
#     data0,info0 = pub.names.getCurAndStep()
#     with open(info0,'w') as fh: 
#         assert topicWatcher.counter == 0 # not done writing
#     import time
#     time.sleep(.1)
#     topicWatcher.update()
#     assert topicWatcher.counter == 1
    
@testme    
def test_Publisher():
    topic = "test_TopicWatcher"
    delTopic(topic)
    pub = FilePublisher(topic,"txt")
    pub.send(StrMessage("hi",dict(a=2)))
    sub = FileSubscriber(topic,"txt",StrMessage)
    message = sub.recv()
    assert message.data == "hi"
    assert message.info["a"] == 2
    
@testme    
def test_Syncer():
    topicA = "test_SyncerA"
    topicB = "test_SyncerB"
    delTopic(topicA)
    delTopic(topicB)
    pubA = FilePublisher(topicA,"txt")
    pubB = FilePublisher(topicB,"txt")
    subA = FileSubscriber(topicA,"txt",FloatMessage)
    subB = FileSubscriber(topicB,"txt",FloatMessage)

    msgsA = [FloatMessage(t,dict(time=t))
             for t in [0,1,2,3]]
    msgsB = [FloatMessage(t,dict(time=t))
             for t in [-1,0,.1,.9,1,2.9,3.1]]
    for msg in msgsA:
        pubA.send(msg)
    for msg in msgsB:
        pubB.send(msg)

    syncer = Synchronizer(subB, subA)
    aNearB = [syncer.recvMulti() for _ in msgsB]
    aData = [msg[1].data for msg in aNearB]
    assert aData == [0,0,0,1,1,3,3]
    

@testme
def test_Live():
    comm.setLive(True)
    comm.setTimeout(1)
    
    topic = "test_Live"
    delTopic(topic)
    pub = FilePublisher(topic, "txt")
    sub = FileSubscriber(topic,"txt",ArrayMessage)
    
    msg = ArrayMessage(data = np.ones((100,100)))
    def func():
        print "publishing"
        pub.send(msg)
        print "done"
    pub_thread = threading.Thread(target=func)
    pub_thread.start()
    
    print "trying to receive"
    msg_recv = sub.recv()
    print "received"
    
    assert (msg_recv.data == msg.data).all()

if __name__ == "__main__":
    test_all(stop=True)
