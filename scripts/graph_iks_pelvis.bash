#!/usr/bin/env bash
set -e

rqt_plot \
/ik_lowerbody_node/ik/joints/pelvis_list \
/ik_lowerbody_node/ik/joints/pelvis_rotation \
/ik_lowerbody_node/ik/joints/pelvis_tilt \

#rqt_plot \
#	/ik/output/data[0] \
#	/ik/output/data[1] \
#	/ik/output/data[2] \
#	/ik/output/data[3] \
#	/ik/output/data[4] \
#	/ik/output/data[5] \
#	/ik/output/data[6] \
#	/ik/output/data[7] \
#	/ik/output/data[8] \
#	/ik/output/data[9] \
#	/ik/output/data[10] \
#	/ik/output/data[11] \
#	/ik/output/data[12] \
#	/ik/output/data[13] \
#	/ik/output/data[14] \
#	/ik/output/data[15] \
#	/ik/output/data[16] \
#	/ik/output/data[17] 

