#!/usr/bin/env python3

import glob
import time
import subprocess
import os

glob_string ="/catkin_ws/Data/**/*_imus_lower.sto" 
#
#sto_files = glob.glob(glob_string, recursive=True)
##sto_files = sorted(glob.glob(glob_string, recursive=True), key=os.path.getmtime) ##doesnt seem to work
sto_files = sorted(glob.glob(glob_string, recursive=True), key=os.path.basename)

timeout_time = 60

#print(sto_files)

for a_file in sto_files:
    print(a_file)
    if "02_ruoli" in a_file:
        print("skipped")
        continue

    time.sleep(3) ## give you time to cancel this for loop
    command=["/catkin_ws/src/osrt_ros/launch/test/t_4/a.bash",a_file]
    
    proc =  subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:

        outs, errs = proc.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        outs, errs = proc.communicate()

    print(outs)

