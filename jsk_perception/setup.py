#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import find_packages


d = generate_distutils_setup(
    packages=find_packages('python'),
    package_dir={'': 'python'},
)

setup(**d)
