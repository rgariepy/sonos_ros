#!/usr/bin/env python

"""
A ROS binding to SoCo, a Python implementation of an unofficial API
for the Sonos wireless speaker system

SoCo: github.com/SoCo/SoCo

TODO: Everything
"""

import roslib; roslib.load_manifest('sonos_ros')
import rospy

from std_srvs.srv import *
from soco import SoCo, discover
from soco.exceptions import SoCoUPnPException

class sonos_ros:

    def __init__(self):
        rospy.init_node('sonos_ros')

        # Discover speakers
        self.speakers_ = {speaker.player_name:speaker for speaker in discover()}

        # Advertise a basic service or two as PoC 
        # TODO: Automate service creation, if possible
        rospy.Service("~play", Empty, self.play)
        rospy.Service("~pause", Empty, self.pause)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            r.sleep()
    
    def play(self, req):
        for speaker in self.speakers_.values():
            speaker.play()
        return EmptyResponse()

    def pause(self, req):
        for speaker in self.speakers_.values():
            try:
                speaker.pause()
            except SoCoUPnPException as e:
                rospy.loginfo(e)
        return EmptyResponse()

if __name__=="__main__":
    try:
        sonos_ros()
    except:
        pass

