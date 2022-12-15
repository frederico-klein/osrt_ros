#!/usr/bin/env bash
set -e

rqt_plot \
	/grf_right/wrench/wrench/torque/x:y:z \
	/grf_left/wrench/wrench/torque/x:y:z 
