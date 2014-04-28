# Software License Agreement (proprietary) 
#
# @author     <rgariepy@clearpathrobotics.com>
# @copyright (c) 2013, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, is not permitted without the
# express permission of Clearpath Robotics.
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['sonos_ros'],
    package_dir={'':'src'},
    scripts=['src/sonos_ros/sonos.py'],
    requires=['std_msgs', 'rospy', 'diagnostic_updater']
)

setup(**setup_args)
