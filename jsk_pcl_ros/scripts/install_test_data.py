#!/usr/bin/env python

import hashlib
import os
import shlex
import shutil
import subprocess

import rospkg


ROS_HOME = os.getenv('ROS_HOME', os.path.expanduser('~/.ros'))


def extract_tgz(filename, cwd):
    cmd = 'tar zxf {}'.format(filename)
    subprocess.call(shlex.split(cmd), cwd=cwd)


def rosbag_decompress(filename):
    cmd = 'rosbag decompress --quiet {}'.format(filename)
    subprocess.call(shlex.split(cmd))


def download_from_gdrive(url, output):
    cmd = 'gdown --quiet {} -O {}'.format(url, output)
    subprocess.call(shlex.split(cmd))


def check_md5sum(filename, md5):
    return hashlib.md5(open(filename, 'rb').read()).hexdigest() == md5


def install_test_data(filename, url, md5, compressed_bag=None):
    """Install test data checking md5 and rosbag decompress if needed.

    @type compressed_bag: ``str``
    @param compressed_bag: relative path from data_dir to compressed bagfile
    """
    rp = rospkg.RosPack()
    ros_data_dir = os.path.join(ROS_HOME, 'test_data')
    if not os.path.exists(ros_data_dir):
        os.mkdir(ros_data_dir)
    data_dir = os.path.join(rp.get_path('jsk_pcl_ros'), 'test_data')
    cache_file = os.path.join(ros_data_dir, filename)
    output = os.path.join(data_dir, filename)
    if os.path.exists(cache_file):
        shutil.copy(cache_file, output)
        extract_tgz(output, cwd=data_dir)
    if os.path.exists(output) and check_md5sum(output, md5):
        print("[jsk_pcl_ros] '{}' is already latest version".format(output))
        return
    print("[jsk_pcl_ros] Installing '{}'".format(output))
    download_from_gdrive(url, output)
    shutil.copy(output, cache_file)
    extract_tgz(output, cwd=data_dir)
    if compressed_bag is not None:
        compressed_bag = os.path.join(data_dir, compressed_bag)
        rosbag_decompress(compressed_bag)
    print("[jsk_pcl_ros] Done installing '{}'".format(output))


def main():
    print('[jsk_pcl_ros] Start installing test data')

    install_test_data(
        filename='2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud.tgz',
        url='https://drive.google.com/uc?id=0B9P1L--7Wd2vSE5XMmJQczl3NWc&export=download',
        md5='4da6242d74950a171e6d4455be87932e',
        compressed_bag='2015-11-04-19-37-29_baxter-kiva-object-in-hand-cloud/vision.compressed.bag',
    )

    print('[jsk_pcl_ros] Done installing test data')


if __name__ == '__main__':
    main()
