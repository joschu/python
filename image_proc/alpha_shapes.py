from collections import defaultdict
import dionysus as di
import numpy as np
from collections import defaultdict
import networkx as nx


def get_concave_hull(xy,alpha):
    complex = di.Filtration()
    di.fill_alpha2D_complex(xy.tolist(), complex)


    d = defaultdict(int)
    for simplex in complex:
        if simplex.data[0] <= alpha:
            verts = list(simplex.vertices)
            if len(verts) == 3:
                i,j,k = sorted(verts)
                d[(i,j)] += 1
                d[(i,k)] += 1
                d[(j,k)] += 1
    G = nx.Graph()
    G.add_edges_from(edge for (edge,count) in d.items() if count==1)
    components = nx.connected_components(G)
    i_largest = np.argmax([len(comp) for comp in components])
    inds = components[i_largest]
    inds_sorted = [inds[0]]
    for _ in xrange(len(inds)-1):
        for nei in G.neighbors(inds_sorted[-1]):
            if len(inds_sorted) ==1 or nei != inds_sorted[-2]:
                inds_sorted.append(nei)
                break
    return xy[inds_sorted], inds_sorted

#plt.figure(2)
#edges = [list(s.vertices) for s in complex if s.data[0] <= .5 and s.data[1] and len(list(s.vertices))==2]
#for (i,j) in edges:
    #plt.plot([x[i,0], x[j,0]],[x[i,1], x[j,1]])
    
#plt.figure(3)
#edges = [list(s.vertices) for s in complex if s.data[0] <= .5 and not s.data[1] and len(list(s.vertices))==2]
#for (i,j) in edges:    
    #plt.plot([x[i,0], x[j,0]],[x[i,1], x[j,1]])    
    
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    plt.clf()
    x = np.random.randn(100,2)
    plt.plot(x[:,0], x[:,1],'b.')
    verts, inds = get_concave_hull(x,.5)
    plt.plot(verts[:,0], verts[:,1],'g',mew=3)
    plt.show()