"""
ROS2 library file for common functions for declaring, importing, and handling publish/subscribe topics

CHANGELOG:
 - Version 1.0.0: Initial release
"""

__author__      = "Braidan Duffy"
__copyright__   = "Copyright 2023"
__credits__     = "Braidan Duffy"
__license__     = "MIT"
__version__     = "1.0.0"
__maintainer__  = "Braidan Duffy"
__email__       = "bduffy2018@my.fit.edu"
__status__      = "Prototype"

from enum import Enum

class TopicNames(str, Enum):
    INERTIAL = "inertial"
    PLATE_MAGNETOMETER = "plate/magnetometer"
    MAGNET_DETECT = "magnet_detect"
    

class PlateDirection(Enum):
    COUNTER_CLOCKWISE = 0
    CLOCKWISE = 1