#!/bin/bash
CALIBFILE=calibrationdata.tar.gz
CALIBLOCAL=calibrationdata_$$.tar.gz

if [ -e /tmp/$CALIBFILE ]; then
    cp /tmp/$CALIBFILE $CALIBLOCAL
else
    exit -1;
fi

LOCDIR=`pwd`
tar xvf $LOCDIR/$CALIBLOCAL ost.txt
LINENUM=(`grep -n -i ost ost.txt | sed -e 's/\([0-9]\+\).*/\1/'`)
#cd $LOCDIR
mv ost.txt ost_$$.txt

if [ ${#LINENUM[@]} -eq 2 ]; then
    split ost_$$.txt -l ${LINENUM[1]}
    mv xaa /tmp/left.ini
    mv xab /tmp/right.ini
    rosrun camera_calibration_parsers convert /tmp/left.ini left_$$.yaml
    rosrun camera_calibration_parsers convert /tmp/right.ini right_$$.yaml
else
    cp ost_$$.txt /tmp/ost.ini
    rosrun camera_calibration_parsers convert /tmp/ost.ini mono_$$.yaml
fi
