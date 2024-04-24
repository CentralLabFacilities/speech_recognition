from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['clf_audio_util'],
    package_dir={'': 'src'}
)

setup(**d)
