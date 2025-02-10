from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["ros_nllb"], package_dir={"": "src"})

setup(**d)
