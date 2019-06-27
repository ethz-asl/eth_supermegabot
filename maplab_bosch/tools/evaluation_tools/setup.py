#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['evaluation_tools'],
    package_dir={'': 'python'},
    install_requires=['yaml'],
)

setup(**d)
