import numpy as np

def read_next_noncomment(f):
    while True:
        line = f.readline()
        if not line.startswith('#'): 
            return line

def read_specific_line(f,wantstr,typ='none'):
    line = read_next_noncomment(f)
    gotstr = line[:len(wantstr)]
    if gotstr == wantstr:
        nextstr = line[len(wantstr):].strip()
        if typ == "float": return float(nextstr)
        elif typ == "int": return int(nextstr)
        elif typ == "str": return nextstr
        elif typ == "floatlist": return map(float,nextstr.split())
        elif typ == "intlist": return map(int,nextstr.split())
        elif typ == "strlist": return nextstr.split()
        elif typ == "none": return
        else: raise Exception('unknown type %s'%typ)
    else:
        raise Exception('wanted %s, got %s'%(wantstr,gotstr))
        
def load_pcd(f):
    if isinstance(f,str) or isinstance(f,unicode):
        with open(f,'r') as fh:
            return load_pcd(fh)
    elif isinstance(f,file):
        read_specific_line(f,'VERSION')
        FIELDS = read_specific_line(f,'FIELDS','strlist')
        SIZE = read_specific_line(f,'SIZE','intlist')
        TYPE = read_specific_line(f,'TYPE','strlist')
        COUNT = read_specific_line(f,'COUNT','intlist')
        WIDTH = read_specific_line(f,'WIDTH','int')
        HEIGHT = read_specific_line(f,'HEIGHT','int')
        read_specific_line(f,'VIEWPOINT')
        POINTS = read_specific_line(f,'POINTS','int')
        DATA = read_specific_line(f,'DATA','str')
        
        typemap = {('F',4):'f4', ('U',4):'u4'} # etc.        
        types = [typemap[pair] for pair in zip(TYPE,SIZE)]
        dtype = np.dtype([(field,typ,count) 
                          for (field,typ,count) in zip(FIELDS,types,COUNT)])        
        if DATA == 'ascii':
            arr = np.loadtxt(f,dtype=dtype)
            return arr[:POINTS].reshape(HEIGHT,WIDTH)
        elif DATA == 'binary':
            arr = np.fromfile(f,dtype=dtype)
            return arr[:POINTS].reshape(HEIGHT,WIDTH)
        else: raise Exception("DATA")
    else:
        raise TypeError
        
def load_xyz(f):
    arr = load_pcd(f)
    xyz = np.concatenate([arr['x'][:,:,None],arr['y'][:,:,None],arr['z'][:,:,None]],2)
    return xyz[0]
    
def load_xyzrgb(f):
    "it's actually xyzbgr"
    arr = load_pcd(f)
    xyz = np.concatenate([arr['x'][:,:,None],arr['y'][:,:,None],arr['z'][:,:,None]],2)
    if 'rgba' in arr.dtype.fields:
        rgb = unpack_rgb(arr['rgba'])
    elif 'rgb' in arr.dtype.fields:
        rgb = unpack_rgb(arr['rgb'])
    else:
        raise Exception("can't find rgb or rgba field")
    return xyz,rgb
    
def unpack_rgb(arr):
    (height,width) = arr.shape
    rgb0 = np.ndarray(buffer=arr.copy(),shape=(height,width,4),dtype='uint8')
    return rgb0[:,:,:3]
