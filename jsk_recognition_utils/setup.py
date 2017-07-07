#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['jsk_recognition_utils', 'jsk_recognition_utils.chainermodels'],
    package_dir={'': 'python'},
)

setup(**d)
