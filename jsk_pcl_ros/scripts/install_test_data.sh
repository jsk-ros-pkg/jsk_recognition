#!/bin/bash

CWD=$(pwd)
TEST_DATA_DIR=$(rospack find jsk_pcl_ros)/test_data

cd $TEST_DATA_DIR

log_file=2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud.tgz
if [ ! -f $log_file ]; then
  gdown "https://drive.google.com/uc?id=0B9P1L--7Wd2vSE5XMmJQczl3NWc&export=download" -O $log_file
  tar zxf 2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud.tgz
  rosbag decompress 2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud/vision.compressed.bag
fi

cd $CWD
