from __future__ import absolute_import
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['sr_utilities'],
    scripts=[],
    package_dir={'': 'scripts'}
)

setup(**d)
