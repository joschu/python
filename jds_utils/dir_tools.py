import os,shutil
from jds_utils.yes_or_no import yes_or_no

def mkdir_ask(path,make_path=False):
    if os.path.exists(path):
        consent = yes_or_no("%s already exists. Delete it?"%path)
        if consent:
            shutil.rmtree(path)
        else:
            raise IOError
    if make_path: os.makedirs(path)
    else: os.mkdir(path)

def ensure_exists(path):
    if not os.path.exists(path):
        os.mkdir(path)
        
def unsafe_reset(path):
    if os.path.exists(path):
        shutil.rmtree(path)
    os.mkdir(path)