#!/usr/bin/env python

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['dasom_control_gui'],
    package_dir={'': 'src'},
    maintainer='Sol Choi',
    maintainer_email='jennychoi0904@gmail.com',
)

setup(**d)
