from __future__ import absolute_import
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sr_tactile_sensors'],
    package_dir={'': 'scripts'})

setup(**setup_args)
