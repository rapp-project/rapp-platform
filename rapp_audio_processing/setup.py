#! /usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup as catkin_setup

dist = catkin_setup(packages=['rapp_audio_processing'], package_dir={'': 'src'})

setup(**dist)
