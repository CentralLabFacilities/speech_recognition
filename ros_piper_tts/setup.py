from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["piper_tts"], package_dir={"": "src"})

setup(**d)
