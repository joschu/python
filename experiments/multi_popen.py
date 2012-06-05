import subprocess, sys


p1 = subprocess.Popen("python ask_name.py 1",  shell=True, stdin = subprocess.STDIN)
p2 = subprocess.Popen("python ask_name.py 2",  shell=True, stdin = subprocess.STDOUT)
