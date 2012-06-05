import argparse
import os, shutil
from glob import glob

parser = argparse.ArgumentParser()
parser.add_argument('dir')
parser.add_argument('From',type=int)
parser.add_argument('To',type=int)
args = parser.parse_args()

start_dir = os.getcwd()
os.chdir(args.dir)

#ext2count = defaultdict(int)
#for file in os.listdir('.'): ext2count[os.path.splitext(file)[1]] += 1
#exts = ext2count.keys()
#counts = ext2count.values()


msgs = sorted(glob('msg*'))
nums = [int(msg[3:15]) for msg in msgs]
assert nums == range(len(msgs))

for msg in msgs[:args.From]:
    os.remove(msg)
for msg in msgs[args.To:]:
    os.remove(msg)
for msg in msgs[args.From:args.To]:
    num = int(msg[3:15])
    shutil.move(msg,msg[:3] + "%.12i"%(num-args.From) + msg[15:])

os.chdir(start_dir)