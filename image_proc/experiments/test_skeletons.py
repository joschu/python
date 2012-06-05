import skeletons
import cv2
import scipy.spatial.distance as ssd

import networkx as nx
import itertools as it
import numpy as np
import matplotlib.pyplot as plt
plt.ion(); plt.clf()

skel = cv2.imread('test_data/ridgeskel.png')[:,:,0]
BigG=skeletons.skel2graph(skel)
Gs = nx.connected_component_subgraphs(BigG)
paths = [skeletons.longest_path(G) for G in Gs]

matching_graph = nx.Graph()
endpoints = [path[0] for path in paths] + [path[-1] for path in paths]
distmat = ssd.squareform(ssd.pdist(endpoints))

for i in xrange(len(endpoints)):
    sortinds = distmat[i].argsort()
    for j in sortinds[1:20]:
        matching_graph.add_edge(endpoints[i],endpoints[j],weight=-distmat[i,j])
    #matching_graph.add_edge(endpoints[i],endpoints[i],weight=0)
    matching_graph.add_edge("start",endpoints[i],weight=0)
    matching_graph.add_edge("end",endpoints[i],weight=0)
        
        
M = nx.matching.max_weight_matching(matching_graph,maxcardinality=True)

for path in paths:
    xys = np.array(path)
    xs,ys = xys.T
    plt.plot(xs,ys)
    
    
    
plt.plot(np.array(endpoints)[:,0],np.array(endpoints)[:,1],'bo')
    
for (src,targ) in M.items():
    if not isinstance(src,str) and not isinstance(targ,str):
        plt.plot([src[0],targ[0]],[src[1],targ[1]],'k:')