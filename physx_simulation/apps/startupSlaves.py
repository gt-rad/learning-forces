import time
import subprocess
import shlex

masterargs = 'Release/policySearch';
masterargs = shlex.split(masterargs)
masterproc = subprocess.Popen(masterargs)
time.sleep(1)
 # get the window handle of the blank, minimized notepad window
for i in range(3):
    slaveargs = 'Release/evaluator' + ' ' + str(i)
    slaveargs = shlex.split(slaveargs)
    print(slaveargs)
    slaveproc = subprocess.Popen(slaveargs)
