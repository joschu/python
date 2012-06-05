import argparse
import comm
import multiprocessing, os
from time import time, sleep

class Mapper(object):
    def __init__(self, inputExts, inputClasses, outputExt, outputClass, inputDescs=None):
        if inputDescs is None: inputDescs = inputExts           
        parser = make_parser(inputDescs)
        self.add_extra_arguments(parser)
        self.args = parser.parse_args()
        
        self.subs = [comm.FileSubscriber(getattr(self.args,desc), ext, cls) for (desc, ext,cls) in zip(inputDescs, inputExts,inputClasses)]
        self.pub = comm.FilePublisher(self.args.output, outputExt)        
        self.outputClass = outputClass
        
    def add_extra_arguments(self,parser):
        return parser

    def func(self,*data):
        raise

    def run(self):
        count = 0
        while True:
            try: 
                inputMsgs = [sub.recv() for sub in self.subs]
            except StopIteration: 
                print "done"
                break

            outputMsg = self.outputClass()
            outputMsg.info = inputMsgs[0].info
            outputMsg.data = self.func(*[inputMsg.data for inputMsg in inputMsgs])

            self.pub.send(outputMsg)
            print "processed inputs %i"%count
            count += 1
            
def make_parser(inputDescs):
    parser = argparse.ArgumentParser()
    if len(set(inputDescs)) < len(inputDescs): raise Exception("two inputs have the same description")        
    for desc in inputDescs: parser.add_argument(desc)
    parser.add_argument("output")
    return parser

class ParallelMapper(object):
    
    def __init__(self, inputExts, inputClasses, outputExt, outputClass, inputDescs=None):
        
        if inputDescs is None: inputDescs = inputExts
        parser = make_parser(inputDescs)
        parser.add_argument("--procs",type=int,default=2)                                
        self.add_extra_arguments(parser)        
        self.args = parser.parse_args()
        
        self.getters = [comm.FileGetter(getattr(self.args,desc), ext, cls) for (desc, ext,cls) in zip(inputDescs, inputExts,inputClasses)]
        self.putter = comm.FilePutter(self.args.output, outputExt)        
        self.outputClass = outputClass    
    
        self.inputExts = inputExts
        self.inputTopics = [getattr(self.args,desc) for desc in inputDescs]
    
        self.input_queue = multiprocessing.Queue()

    def add_extra_arguments(self, parser):
        return parser
     
        
    def start_workers(self):
        self.workers = [multiprocessing.Process(target = self.worker_loop, args=(i,)) for i in xrange(self.args.procs)]
        print "starting %i workers"%self.args.procs
        self.workers[0].start()
        sleep(.5)
        for worker in self.workers[1:]: worker.start()
        
        
    def run(self):
        self.start_workers()
        id = 0
        while True:
            info_names =  [comm.makeNamePair(id, extension, topic)[1] for (extension, topic) in zip(self.inputExts, self.inputTopics)]
            
            t_start_wait = time()
            while True:
                if all(os.path.exists(info_name) for info_name in info_names): break
                else: sleep(.01)
            self.input_queue.put(id)
            id += 1
                    
    def shutdown(self):
        for worker in self.workers:
            worker.terminate()
                    
    def worker_loop(self, worker_num):
        print "worker %i starting"%worker_num
        while True:
            id = self.input_queue.get()
            inputMsgs = [getter.recv_id(id) for getter in self.getters]
            
            outputMsg = self.outputClass()
            outputMsg.info = inputMsgs[0].info
            outputMsg.data = self.func(*[inputMsg.data for inputMsg in inputMsgs])

            self.putter.send_id(outputMsg,id)
            print "worker %i processed input %i"%(worker_num, id)
            
        
    def func(self, *data):
        raise

