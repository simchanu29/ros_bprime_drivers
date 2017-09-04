## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

# http://wiki.ros.org/rospy_tutorials/Tutorials/Makefile

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['adafruit_pwm_drivers'],
    package_dir={'': 'include'},
)

setup(**setup_args)