#!/usr/bin/env python

from setuptools import setup

from setuptools import find_packages
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    # Uncomment until src/jsk_recognition_msgs
    # error: package directory 'jsk_recognition_msgs' does not exist
    # [jsk_recognition_msgs:install]
    # packages=['jsk_recognition_msgs'],
    # [jsk_recognition_msgs:install] error: package directory 'src/jsk_recognition_msgs' does not exist
    # package_dir={'': 'src'},
)

setup(**d)
