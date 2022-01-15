from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["utm","airsim"],
    package_dir={"":"src"}
)

setup(**d)