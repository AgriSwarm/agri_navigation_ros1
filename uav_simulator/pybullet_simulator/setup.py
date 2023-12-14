from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['gym_pybullet_drones'],
    package_dir={'': 'src/pkg'},
)

setup(**d)