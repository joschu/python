from comm import comm, mapper
import subprocess, os, shutil
import jds_utils.dir_tools

comm.setDataRoot("/home/joschu/Data/test_comm")
dir_tools.ensure_exists(comm.getDataRoot())


def delTopic(topic):
    path = comm.topicPath(topic)
    if os.path.exists(path):
        shutil.rmtree(path)
# 
 
topicA = "topicA"
topicB = "topicB"
delTopic(topicA)
delTopic(topicB)
pubA = comm.FilePublisher(topicA,"txt")
pubB = comm.FilePublisher(topicB,"txt")
msgsA = [comm.FloatMessage(t) for t in [0,1,2,3]]
msgsB = [comm.FloatMessage(t) for t in [-1,0,.1,.9]]
for msg in msgsA:
    pubA.send(msg)
for msg in msgsB:
    pubB.send(msg)
    
class TestParMap(mapper.ParallelMapper):
    def __init__(self):
        mapper.ParallelMapper.__init__(self, ["txt","txt"],[comm.FloatMessage, comm.FloatMessage],"txt",comm.FloatMessage, inputDescs = ["input_topic1", "input_topic2"])
    def func(self, dataA, dataB):
        return dataA+dataB

delTopic("topicC")
mapper = TestParMap()
mapper.run()