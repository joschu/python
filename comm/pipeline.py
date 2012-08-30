import networkx as nx
from os.path import isdir, join, exists, basename
from glob import glob
import matplotlib.pyplot as plt
from time import sleep, time
from . import comm
import subprocess, signal
from jds_utils.colorize import colorize
import traceback, os, logging, json

class Pipeline(object):
    env = {}
    def __init__(self):
        self.graph = nx.DiGraph()
    def add_topic(self, topic, **kw):
        self.graph.add_node(topic,type="topic",**kw)
    def add_file(self, file):
        self.graph.add_node(file,type="file")
    def add_program(self, program, command, dependencies, products):
        self.graph.add_node(program,type="program",command=command)
        for item in dependencies: 
            if not self.graph.has_node(item): raise Exception("you forgot to add item %s"%item)
            self.graph.add_edge(item, program)
        for item in products: 
            if not self.graph.has_node(item): raise Exception("you forgot to add item %s"%item)            
            self.graph.add_edge(program, item)
            
    def restrict_to_target(self, target):
        revgraph = self.graph.reverse()
        nodes = nx.dfs_postorder(revgraph, target)
        new_graph = self.graph.subgraph(nodes)
        new_pipeline = Pipeline()
        new_pipeline.graph = new_graph
        new_pipeline.env = self.env
        return new_pipeline
    def get_all_status(self):
        return dict([(item,self.get_status(item)) for item in self.get_items()])
    def get_status(self,item):
        if self.graph.node[item]["type"] == "topic":
            if exists(item): return data_nums(item)
            else: return None
        elif self.graph.node[item]["type"] == "file":
            if exists(item): return 1
            else: return 0
    def get_items(self):
        return [node for node in self.graph.nodes() if self.graph.node[node]["type"] in ["topic","file"]]
    def get_programs(self):
        return [node for node in self.graph.nodes() if self.graph.node[node]["type"] == "program"]
    def get_topics(self):
        return [node for node in self.graph.nodes() if self.graph.node[node]["type"] == "topic"]
    def get_source_topic(self):
        return "kinect"
        # for topic in nx.topological_sort(self.graph): 
        #     if self.graph.node[topic]["type"] == "topic" and not (: return topic
    def get_target_topic(self):
        return nx.topological_sort(self.graph)[-1]
    def ready_to_run(self, program, item2status):
        item2status = self.get_all_status()
        return all(bool(item2status[pred]) for pred in self.graph.predecessors(program))
    def products_already_made(self, program, item2done):
        return all(item2done[item] for item in self.graph.successors(program))
    def makes_topic(self, program):
        return any(self.graph.node[succ]["type"] == "topic" for succ in self.graph.successors(program))
    def get_all_doneness(self, item2status):
        source_topic = self.get_source_topic()
        source_count = max(item2status[source_topic]) if bool(item2status[source_topic]) else 0
        item2done = {}
        for (item, status) in item2status.items():
            if self.graph.node[item]["type"] == "topic":
                item2done[item] = bool(item2status[item]) and max(item2status[item]) == source_count
            elif self.graph.node[item]["type"] == "file":
                item2done[item] = bool(item2status[item])
        return item2done
    def cleanup_all(self, lifetime):
        target_topic = self.get_target_topic()
        item2status = self.get_all_status()
        max_id_target = status2count(item2status[target_topic]) - 1
        min_id = max(max_id_target-lifetime,0)                
        if max_id_target == -1: return
        
        for topic in self.get_topics():
            if item2status[topic] and not self.graph.node[topic].get("dont_clean"):
                
                if not self.graph.node[topic].get("extension"):
                    self.graph.node[topic]["extension"] = glob(comm.filePath("data0*",topic))[0].split('.')[-1]
                                
                
                if self.graph.node[topic].get("async"):
                    dataFiles = glob(comm.filePath("data*",topic))
                    infoFiles = glob(comm.filePath("info*",topic))
                    lastTargetInfoFile = comm.makeNamePair(max_id_target, "ass", target_topic)[1]
                    with open(lastTargetInfoFile,"r") as fh: info = json.load(fh)
                    lastTargetTime = info["time"]

                    for (dataName, infoName) in zip(dataFiles,infoFiles):
                        if lastTargetTime - os.stat(infoName).st_mtime > 10: #10 seconds
                            logging.info("deleting %s",dataName)
                            logging.info("deleting %s",infoName)
                            os.unlink(dataName)
                            os.unlink(infoName)
                            
                    
                else:
                    for id in item2status[topic]:
                        if id < min_id:
                            dataName, infoName = comm.makeNamePair(id, self.graph.node[topic]["extension"], topic)                            
                            logging.info("deleting %s",dataName)
                            logging.info("deleting %s",infoName)
                            os.unlink(dataName)
                            os.unlink(infoName)



    def throttle(self, target_topic, item2status, max_lag):
        source_topic = self.get_source_topic()
        comm.setThrottled(source_topic,
                           status2count(item2status[source_topic]) - status2count(item2status[target_topic]) > max_lag)

def status2count(status):
    "status = None | [Int]"
    return 0 if bool(status)==False else max(status)

class PipelinePlot(object):
    first_draw = True
    def __init__(self, pipeline):
        self.pipeline = pipeline
        self.graph = pipeline.graph
        type2color = dict(file = "orange", topic="red", program="blue")
        self.colors = [type2color[self.graph.node[node]["type"]] for node in self.graph.nodes()]
        self.positions = nx.graphviz_layout(self.graph, prog="dot")
        self.labels = dict(zip(self.graph.nodes(), self.graph.nodes()))
        
    def draw(self, item2status = None):
        if item2status is None:
            item2status = self.pipeline.get_all_status()
        self.update_labels(item2status)
        
        if self.first_draw:
            nx.draw_networkx(self.graph, pos = self.positions, labels = self.labels, node_color = self.colors, nodelist = self.graph.nodes())
            self.first_draw = False
        else:
            for text_obj in plt.gca().texts:
                maybe_node = text_obj.get_text().split()[0]
                if self.labels.has_key(maybe_node): text_obj.set_text(self.labels[maybe_node])
            
    def update_labels(self, item2status):
        for (item, status) in item2status.items():
            if status is None: status_str = "-"
            elif status == []: status_str = "0"
            elif isinstance(status, list): status_str = "%i: %i"%(min(status),max(status))
            else: status_str = str(status)
            self.labels[item] = "%s\n%s"%(item,status_str)
            
    def loop(self):
        plt.ion()
        while True:        
            self.draw()    
            plt.draw()
            sleep(.1)

        
    
def execute_series(pipeline, dry_run=False):
    nodedict = pipeline.graph.node
    ordered_progs = [node for node in nx.topological_sort(pipeline.graph) if nodedict[node]["type"] == "program"]
    for prog in ordered_progs:
        command = nodedict[prog]["command"]
        item2status = pipeline.get_all_status()
        if pipeline.products_already_made(prog, item2status):
            print "next:", colorize("skipping %s"%prog, "red")
            logging.info("skipping %s",prog)
            continue        
        print colorize(command,"red")
        logging.info(command)
        raw_input("press enter to continue")        
        if not dry_run: 
            child = subprocess.Popen(command.split(), env=pipeline.env)
            try:
                interrupted=False
                while child.poll() is None: sleep(.1)
            except KeyboardInterrupt:
                interrupted=True
            if not interrupted and child.returncode != 0:
                raise subprocess.CalledProcessError(child.returncode, command)
            #try:
                #while child.poll():
                    #sleep(1)
            #except KeyboardInterrupt:
                #print "got signal. terminating subprocess"
                #child.send_signal(signal.SIGINT)
                
def execute_parallel(pipeline, lifetime, max_lag, noclean = False):
    nodedict = pipeline.graph.node
    target_topic = pipeline.get_target_topic()
    prog2child = {}
    remaining_progs = pipeline.get_programs()
    #plot = PipelinePlot(pipeline)

    try:
        while True:
            item2status = pipeline.get_all_status()
            item2done = pipeline.get_all_doneness(item2status)
            remaining_progs = [prog for prog in remaining_progs if not pipeline.products_already_made(prog, item2done) or pipeline.makes_topic(prog)]
            for prog in remaining_progs:
                if pipeline.ready_to_run(prog, item2status):
                    remaining_progs.remove(prog)
                    command = nodedict[prog]["command"]
                    print colorize(command, "red")                
                    logging.info(command)
                    child = subprocess.Popen(command.split(), env=pipeline.env)
                    prog2child[prog] = child
            for (prog, child) in prog2child.items():
                if child.poll() is not None and child.returncode != 0:
                    print colorize("%s failed"%prog, "red")
                    logging.error("%s failed",prog)
                    for child in prog2child.values():
                        if child.poll() is None: child.terminate()
                    return
            if not noclean: pipeline.cleanup_all(lifetime)
            pipeline.throttle(target_topic, item2status, max_lag)
            #plot.draw()
            sleep(.1)
    except Exception:
        traceback.print_exc()
        for child in prog2child.values(): 
            if child.poll() is None: child.terminate()
            return
            
    
    
def data_nums(topic):
    fnames = glob(join(topic,"info*.json"))
    nums = sorted([int(basename(fname)[4:16]) for fname in fnames])
    return nums
    
