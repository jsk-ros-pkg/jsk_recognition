#!/usr/bin/env python

from setuptools import setup

from setuptools import find_packages
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['audio_to_spectrogram'],
    package_dir={'': 'src'},
)

setup(**d)
