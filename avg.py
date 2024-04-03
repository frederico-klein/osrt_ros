#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty

from threading import Lock

rospy.init_node("avg")

sub_list = []
pub = rospy.Publisher("/ik/heading_angle",Float64,latch=True)

heading = None
headings = None

average_heading_lock = Lock()
headings_lock = Lock()

def clear_headings(req):
    global headings
    with headings_lock:
        headings = {
            "pelvis":None,
            "torso":None,
            "femur_r":None,
            "tibia_r":None,
            "talus_r":None,
            "femur_l":None,
            "tibia_l":None,
            "talus_l":None,
                }
    return True

def callback(msg, args):
    #print(args)
    #print(msg)
    global headings
    with headings_lock:
        headings[args["body"]] = msg.data
    print(headings)
    n=0
    sum = 0
    for head_, ang in headings.items():
        if ang:
            n+=1
            sum+=ang
    ff = Float64()
    if n>0:
        global heading
        with average_heading_lock:
            heading = sum/n
        ff.data = sum/n
        print(ff.data*180/3.151492)
        pub.publish(ff)

clear_headings(None)

for head_, val in headings.items():
    #sub_list.append(rospy.Subscriber("/ik/%s/heading_angle"%head_,Float64,callback=callback,callback_args={"1":1,"2":2}))
    sub_list.append(rospy.Subscriber("/ik/%s/heading_angle"%head_,Float64,callback=callback,callback_args={"body":head_}))


## another service to clear headings
rospy.Service("clear", Empty, clear_headings)

rospy.spin()
