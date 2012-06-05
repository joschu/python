import functools

class once:
    def __init__(self,fn):
        self.fn = fn
        self.out = None    
    def __call__(self,*args,**kw):
        if self.out is None:
            self.out = self.fn(*args,**kw)
        return self.out
        
def disp_args(*args,**kw):
    return ",".join([str(arg) for arg in args] + ["%s=%s"%(str(key),str(val)) for (key,val) in kw.items()])        
        
TAB_LEVEL = 0
def verbose(fn):
    @functools.wraps(fn)
    def new_ver(*args,**kw):
        global TAB_LEVEL
        print("\t"*TAB_LEVEL+"%s(%s)"%(fn.__name__,disp_args(*args,**kw)))
        TAB_LEVEL += 1
        result = fn(*args,**kw)
        TAB_LEVEL -= 1            
        print("\t"*TAB_LEVEL+"=> %s"%str(result))    
        return result
    return new_ver        