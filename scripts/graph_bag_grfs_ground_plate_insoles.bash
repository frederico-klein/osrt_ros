#!/usr/bin/env bash
set -e

rqt_plot \
	/grf_right/wrench/wrench/force/y \
	/grf_left/wrench/wrench/force/y \
	/right/wrench/wrench/force/y \
	/left/wrench/wrench/force/y \

#rqt_plot \
#	/grf_right/wrench/wrench/force/x:y:z \
#	/grf_right/wrench/wrench/torque/x:y:z \
#	/grf_left/wrench/wrench/force/x:y:z \
#	/grf_left/wrench/wrench/torque/x:y:z \
#	/right/wrench/wrench/force/x:y:z \
#	/right/wrench/wrench/torque/x:y:z \
#	/left/wrench/wrench/force/x:y:z \
#	/left/wrench/wrench/torque/x:y:z \

