from jds_utils.shell import call_and_print
import os, tarfile
from glob import glob

def glob1(pat):
    files = glob(pat)
    if len(files) != 1: raise Exception("pattern %s has the wrong number of matches: %s"%(pat, str(files)))
    else: return files[0]
    
class Dummy:
    def add(*args,**kw):
        pass
    
    
os.chdir("/home/joschu/Data/tracking_dataset")
with tarfile.open("robot_rope.tar.gz", 'w:gz') as f:
    #f = Dummy()
    
    dirs = ["/home/joschu/Data/tracking_results/robotjs_tie_overhand_2012-09-16-00-12-52",
            "/home/joschu/Data/tracking_results/robotjs_tie_figure8_2012-09-15-23-22-47",
            "/home/joschu/Data/tracking_results/robotjs_tie_figure8_2012-09-15-23-49-14"]
    names = ["overhand", "figure8_I", "figure8_II"]
    
    
    for (dir, name) in zip(dirs, names):
        print name
        os.chdir(dir)
        f.add(glob1("*.bag"), 
              arcname="robot_rope/%s_raw.bag"%name)
        f.add(glob1("preproc/sync*.bag"), 
              arcname="robot_rope/%s_preprocessor.bag"%name)
        f.add(glob1("tracking_color/*.bag"),
              arcname="robot_rope/%s_1cam_color.bag"%name)
        f.add(glob1("tracking_nocolor/*.bag"),
              arcname="robot_rope/%s_1cam_nocolor.bag"%name)
