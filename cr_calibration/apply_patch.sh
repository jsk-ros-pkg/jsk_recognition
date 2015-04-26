#!/usr/bin/env bash

cd `rospack find camera_calibration`;
if [ `grep -c rational nodes/cameracalibrator.py` == "0" ]; then
    patch -p0 < `rospack find cr_calibration`/diamondback_camera_calibration.patch;
fi
