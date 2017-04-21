#!/usr/bin/env python

import os.path as osp
import subprocess
import sys

import tabulate


def get_package_table():
    packages = [
        'jsk_recognition_msgs',
        'jsk_perception',
        'jsk_pcl_ros',
        'jsk_pcl_ros_utils',
        'resized_image_transport',
        'jsk_recognition_utils',
        'checkerboard_detector',
        'imagesift',
    ]

    headers = ['Package', 'Description', 'Documentation', 'Code']
    rows = []
    for pkg in packages:
        from bs4 import BeautifulSoup
        soup = BeautifulSoup(open(osp.join(pkg, 'package.xml')).read())
        website_url = soup.find('url', type='website')
        if website_url:
            doc_url = '[![](https://img.shields.io/badge/docs-here-brightgreen.svg)](%s)' % website_url.text
        else:
            doc_url = ''
        repo_url = soup.find('url', type='repository')
        if repo_url:
            code_url = '[![](https://img.shields.io/badge/code-here-brightgreen.svg)](%s)' % \
                osp.join(repo_url.text, 'tree/master', pkg)
        else:
            code_url = ''
        desc = filter(None, soup.find('description').text.splitlines())[0].strip()
        desc = desc[:50]
        if not desc.endswith('.'):
            desc += '...'
        row = [pkg, desc, doc_url, code_url]
        rows.append(row)
    return tabulate.tabulate(rows, headers=headers, tablefmt='pipe')


def get_deb_status_table():
    cmd = 'rosrun jsk_tools generate_deb_status_table.py jsk_recognition -f i -t k'
    return subprocess.check_output(cmd, shell=True).strip()


template = '''\
jsk\_recognition
===============

[![GitHub version](https://badge.fury.io/gh/jsk-ros-pkg%2Fjsk_recognition.svg)](https://badge.fury.io/gh/jsk-ros-pkg%2Fjsk_recognition)
[![Build Status](https://travis-ci.org/jsk-ros-pkg/jsk_recognition.svg)](https://travis-ci.org/jsk-ros-pkg/jsk_recognition)
[![Read the Docs](https://readthedocs.org/projects/jsk-docs/badge/?version=latest)](http://jsk-docs.readthedocs.org/en/latest/jsk_recognition/doc/index.html)

jsk_recognition is a stack for the perception packages which are used in JSK lab.


Deb build status
----------------

{DEB_STATUS_TABLE}


ROS packages
------------

{PACKAGES_TABLE}


Deprecated packages
-------------------
* [cr\_calibration](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/cr_calibration)
* [cr\_capture](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/cr_capture)
* [orbit\_pantilt](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/orbit_pantilt)
* [posedetectiondb](https://github.com/jsk-ros-pkg/jsk_recognition/tree/master/posedetectiondb)
'''


def main():
    sys.stdout.write(template.format(DEB_STATUS_TABLE=get_deb_status_table(),
                                     PACKAGES_TABLE=get_package_table()))


if __name__ == '__main__':
    main()
