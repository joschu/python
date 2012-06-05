import numpy as np

def test_pca():
    x = np.random.randn(100,2)
    xc = x - x.mean(axis=0)
    u,d,vh = np.linalg.svd(xc)
    
    pc1a = vh[0]
    
    vals,vecs = np.linalg.eigh(np.cov(x.T))
    pc1b = vecs[:,-1]
    
    print pc1a
    print pc1b
    
def pca(x_tk,n_pc):
    xc_tk = x_tk - x_tk.mean(axis=0)
    u,d,vh = np.linalg.svd(x_tk,full_matrices = False)
    return vh[:n_pc]

def pca2(x_tk,n_pc):
    xc_tk = x_tk - x_tk.mean(axis=0)
    u,d,vh = np.linalg.svd(x_tk,full_matrices = False)
    pcs_nk = vh[:n_pc]
    return pcs_nk,np.dot(x_tk,pcs_nk.T)
    