import dionysus as di
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict
plt.close('all')
x = np.random.rand(10,2)


complex = di.Filtration()
di.fill_alpha2D_complex(x.tolist(), complex)
alphashape = [s for s in complex if s.data[0] <= .5]

plt.figure(1)

d = defaultdict(int)
for simplex in complex:
    if simplex.data[0] <= .1:
        verts = list(simplex.vertices)
        if len(verts) == 3:
            i,j,k = sorted(verts)
            d[(i,j)] += 1
            d[(i,k)] += 1
            d[(j,k)] += 1
for (edge, count) in d.items(): 
    if count ==1:
        i,j = edge
        edge_pts=np.array(x[[i,j]])
        plt.plot(edge_pts[:,0], edge_pts[:,1],'b')
        plt.annotate(str(count), edge_pts.mean(axis=0))
    


#plt.figure(2)
#edges = [list(s.vertices) for s in complex if s.data[0] <= .5 and s.data[1] and len(list(s.vertices))==2]
#for (i,j) in edges:
    #plt.plot([x[i,0], x[j,0]],[x[i,1], x[j,1]])
    
#plt.figure(3)
#edges = [list(s.vertices) for s in complex if s.data[0] <= .5 and not s.data[1] and len(list(s.vertices))==2]
#for (i,j) in edges:    
    #plt.plot([x[i,0], x[j,0]],[x[i,1], x[j,1]])    
    
plt.show()