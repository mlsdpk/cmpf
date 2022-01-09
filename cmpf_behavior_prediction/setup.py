from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['cmpf_behavior_prediction'],
    package_dir={'': 'src'}
)

setup(**d)
