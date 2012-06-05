import networkx as nx
import numpy as np
import itertools as it
import scipy.spatial.distance as ssd
import utils
#import morph
import mlabraw_image_processing as mip

# PARAMETERS
WEIGHTS = np.r_[3, 3, 3, 30,200, 200, 30, 200, 200]
MIN_SEG_LEN = 10
PATH_SNIP = 4
MAX_COST = 100
SKIP_PIX_COST = 5
MAX_NEIGHBORS=5

STEPS4 = [(1,-1),(1,0),(1,1),(0,1)]
STEPS2 = [(0,1),(1,0)]

def skel2graph(bwthin):
    G = nx.Graph()    
    us, vs = np.nonzero(bwthin)
    for (u,v) in zip(us,vs):
        for (du,dv) in STEPS4:
            if bwthin[u+du,v+dv]:
                G.add_edge((u,v),(u+du,v+dv))
    return G
    
def removeSpurs(G,minBranchSize):
    deg = G.degree()
    deg1nodes = [node for node in G.nodes() if deg[node]==1]
    
    to_delete = []
    for node in deg1nodes:
        #print "node:",node
        cur_node = node
        prev_node = None
        chain = []
        for i in xrange(minBranchSize):
            #print cur_node,prev_node
            if deg[cur_node] > 2: 
                print "hit",cur_node,"deleting",chain
                to_delete.extend(chain)
            else: 
                chain.append(cur_node)
                for neighb in G[cur_node]:
                    if neighb != prev_node:
                        prev_node = cur_node
                        cur_node = neighb
                        break        
    print "to_delete",to_delete
    G.remove_nodes_from(to_delete)
    
            
def removeSpurs1(bw, minBranchSize):
    pruned = morph.mmthin(bw, morph.mmendpoints("homotopic"),minBranchSize)
    trimmings = bw - pruned
    big_trimmings = morph.mmareaopen(trimmings,minBranchSize)
    small_trimmings = trimmings - big_trimmings
    #import cv2
    #cv2.imshow('hi',small_trimmings.astype('uint8')*100)
    #cv2.waitKey(10)
    return bw - small_trimmings
   
def removeDeg3AndUp(G):
    badnodes = [node for node in G.nodes() if G.degree(node) > 2]
    for node in badnodes: G.remove_node(node)
        
def longest_shortest_path(G):
    ends = [node for (node,deg) in G.degree().items() if deg == 1]

    best_length = 0
    best_path = None    
    for (end0,end1) in it.combinations(ends,2):
        path = nx.shortest_path(G,source=end0,target=end1)
        length = len(path)

        if length > best_length:
            best_length = length
            best_path = path
            
    return best_path
        
def edge_matching(cost_matrix, max_cost, max_neighbors, skip_costs):
    "cost matrix is 2n x 2n"
    M,N = cost_matrix.shape
    assert M==N and M%2==0    
    
    matching_graph = nx.Graph()
    for i in xrange(N):
        sortinds = cost_matrix[i].argsort()
        for j in sortinds[:max_neighbors]:
            if cost_matrix[i,j] < max_cost:
                matching_graph.add_edge(i,j,weight = -cost_matrix[i,j])
        matching_graph.add_edge("start", i, weight=0)
        matching_graph.add_edge("end", i, weight=0)
        
    for i in xrange(N,2):
        matching_graph.add_edge(i,i+1, weight=-skip_costs[i])
    print matching_graph.edges()
        
    M = nx.matching.max_weight_matching(matching_graph, maxcardinality=True)
    return M

def make_path_graph(cost_matrix, lengths):
    M,N = cost_matrix.shape
    assert M==N and M%2==0    
    
    path_graph = nx.Graph()
    for i in xrange(N):
        sortinds = cost_matrix[i].argsort()
        for j in sortinds[:MAX_NEIGHBORS]:
            if i!=j and cost_matrix[i,j] < MAX_COST:
                path_graph.add_edge(i,j,weight = -cost_matrix[i,j])
    for i in xrange(0,N,2):
        if not path_graph.has_node(i): path_graph.add_node(i)
        if not path_graph.has_node(i+1): path_graph.add_node(i+1)
        path_graph.add_edge(i,i+1,weight=lengths[i/2]*SKIP_PIX_COST)
    return path_graph
    

def calc_path_features(pos_a, dir_a, pos_b, dir_b):
    "dir_a points away from a"
    ang_a_disp = ang_between(dir_a, pos_a - pos_b)
    ang_b_disp = ang_between(dir_b, pos_b - pos_a)
    ang_a_b = ang_between(-dir_a, dir_b)
    fwddist_b_a = np.clip(np.dot(pos_b - pos_a, -dir_a),0,np.inf)
    backdist_b_a = -np.clip(np.dot(pos_b - pos_a, -dir_a),-np.inf,0)
    perpdist_b_a = np.sqrt(sqnorm(pos_b - pos_a) - np.dot(pos_b - pos_a, -dir_a)**2)
    fwddist_a_b = np.clip(np.dot(pos_a - pos_b, -dir_b),0,np.inf)
    backdist_a_b = -np.clip(np.dot(pos_a - pos_b, -dir_b),-np.inf,0)    
    perpdist_a_b = np.sqrt(sqnorm(pos_a - pos_b) - np.dot(pos_a - pos_b, -dir_b)**2)
    return np.array([
        ang_a_disp,
        ang_b_disp,
        ang_a_b,
        fwddist_b_a,
        backdist_b_a,
        perpdist_b_a,
        fwddist_a_b,
        backdist_a_b,
        perpdist_a_b])



def sqnorm(x):
    return (x**2).sum()
norm = np.linalg.norm    
def ang_between(x,y):
    return np.arccos(np.dot(x,y)/norm(x)/norm(y))
def normalized(x):
    return x / norm(x)

def start_pos_dir(path):
    return path[0], normalized(path[min(len(path)-1,5)] - path[0])
def end_pos_dir(path):
    return start_pos_dir(path[::-1])

def make_cost_matrix(paths):
    pos_and_dirs = []
    for path in paths:
        pos_and_dirs.append(start_pos_dir(path))
        pos_and_dirs.append(end_pos_dir(path))

    N = len(paths)
    cost_matrix = np.zeros((2*N, 2*N))
    for (i,j) in it.combinations(range(2*N),2):
        pdi = pos_and_dirs[i]
        pdj = pos_and_dirs[j]
        features = calc_path_features(pdi[0],pdi[1], pdj[0],pdj[1])
        cost_matrix[i,j] = cost_matrix[j,i] =  np.dot(WEIGHTS, features)
        print i,j, features
        #cost_matrix[i,j] = cost_matrix[j,i] =  np.linalg.norm(pdi[0] - pdj[0])*1000
        
    return cost_matrix
        
        
def longest_path(G):
    lengths_paths = []
    for start in G.nodes():
        lengths_paths.append(longest_path_from(G,start,set([])))
        
    return max(lengths_paths)[1]


def get_paths_2d(skel):
    BigG=skel2graph(skel)
    #removeSpurs(BigG,10)
    removeDeg3AndUp(BigG)
    Gs = nx.connected_component_subgraphs(BigG)
    paths2d = filter(None,[longest_shortest_path(G) for G in Gs])  
    paths2d = filter(lambda x: len(x) > MIN_SEG_LEN, paths2d)
    paths2d = map(lambda x: x[PATH_SNIP:-PATH_SNIP],paths2d)
    return paths2d



# @verbose
def longest_path_from(G,start,used):  
    global TAB_LEVEL
    if VERBOSE:
        print TAB_LEVEL*'\t', 
        print start,used
        TAB_LEVEL+=1    
    opp = start - 1 if start%2==1 else start+1
    thislength, thispath = (G.edge[start][opp]["weight"],[start,opp])
    lengths_paths = [(thislength, thispath)]
    newused = used.union([start,opp])
    for nei in G.neighbors_iter(opp):
        if nei not in newused:
            neilength,neipath = longest_path_from(G,nei,newused)
            lengths_paths.append(
                (thislength+G.edge[opp][nei]["weight"]+neilength,thispath+neipath))
    if VERBOSE:
        TAB_LEVEL -= 1
        print TAB_LEVEL*'\t', 
        print "-->",max(lengths_paths)
        
    return max(lengths_paths)
    

#@utils.verbose
def longest_path_between(G,start,used,target):  
    
    opp = start - 1 if start%2==1 else start+1
    thislength, thispath = (G.edge[start][opp]["weight"],[start,opp])
    
    if opp == target:
        return (thislength, thispath)
    
    else:
        lengths_paths = []
        newused = used.union([start,opp])
        for nei in G.neighbors_iter(opp):
            if nei not in newused:
                neilength,neipath = longest_path_between(G,nei,newused,target)
                if neilength is not None:
                    lengths_paths.append(
                        (thislength+G.edge[opp][nei]["weight"]+neilength,thispath+neipath))

    if len(lengths_paths) > 0:
        return max(lengths_paths)    
    else: 
        return None,None