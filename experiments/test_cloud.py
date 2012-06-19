import cloud
import xmlrpclib


def matrix_multiply_test():
    import numpy as np
    from time import time
    x = np.zeros((1000,1000))
    t_start = time()
    np.dot(x,x)
    t_elapsed = time() - t_start

    s = xmlrpclib.ServerProxy('http://localhost:8001')
    s.add(2,3)

    return t_elapsed
    

print "local computation time:", matrix_multiply_test()

from time import time    


for i in xrange(1):
    jid = cloud.call(matrix_multiply_test, _type='c2')
    t_start_req = time()
    print "cloud computation time:", cloud.result(jid)    
    print "total request time:", time() - t_start_req
