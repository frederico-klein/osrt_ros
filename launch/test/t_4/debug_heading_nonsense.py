#!/usr/bin/env python3

import glob
import time
import subprocess
import os
import rospkg
rospack = rospkg.RosPack()

osrt_pkg_path = rospack.get_path("osrt_ros")

glob_string ="/srv/host_data/**/*_imus_lower.sto" 
#
#sto_files = glob.glob(glob_string, recursive=True)
##sto_files = sorted(glob.glob(glob_string, recursive=True), key=os.path.getmtime) ##doesnt seem to work
sto_files = sorted(glob.glob(glob_string, recursive=True), key=os.path.basename)

timeout_time = 60

#print(sto_files)
i = 0
#angles_opensimrt = [63.02, -117.72]
angles =[-60.87,-120.2]
## it said -60 now!! 120.15

file_to_run="b"
#file_to_run="a"

for a_file in sto_files:
    print(a_file)
    if True and "ruoli" in a_file:
        print("skipped")
        continue

    if False and not "2023-03-03-11-56-24walking012_imus_lower" in a_file:
        print("skipped")
        continue
    
    time.sleep(3) ## give you time to cancel this for loop
    command=["%s/launch/test/t_4/%s.bash"%(osrt_pkg_path,file_to_run),a_file, str(angles[0])]
    
    proc =  subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    try:

        outs, errs = proc.communicate(timeout=15)
    except subprocess.TimeoutExpired:
        proc.kill()
        outs, errs = proc.communicate()

    print(outs)
    i+=1
    if i>10:
        break
