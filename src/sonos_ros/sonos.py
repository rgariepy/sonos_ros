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
from sonos_ros.srv import *
from soco import SoCo, discover
from soco.exceptions import SoCoUPnPException

class sonos_ros:

    def __init__(self):
        rospy.init_node('sonos_ros')

        # Discover speakers
        self.speakers_ = {speaker.player_name:speaker for speaker in discover()}
        rospy.loginfo("Rooms detected:")
        for speaker in self.speakers_.keys():
            rospy.loginfo(speaker)

        # Advertise a basic service or two as PoC 
        # TODO: Automate service creation, if possible
        rospy.Service("~play", Empty, self.play)
        rospy.Service("~pause", Empty, self.pause)
        rospy.Service("~party", OneArg, self.party)
        rospy.Service("~unparty", Empty, self.unparty)
        rospy.Service("~copy", TwoArg, self.copy_music)
        rospy.Service("~move", TwoArg, self.move_music)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            r.sleep()

    def party(self, req):
        # TODO: Better acknowledgement
        # In SONOS-land, we are joining the destination speaker to the origin
        # (for all destination speakers)
        from_zone = req.zone
        rospy.loginfo("Starting party mode based on %s", from_zone)
        try:
            master_uid = self.speakers_[from_zone].get_speaker_info()["uid"] 
            for speaker in self.speakers_.values():
                if speaker.player_name == from_zone:
                    continue
                speaker.join(master_uid)
            # We're finished. Play.
            self.speakers_[from_zone].play()
        except KeyError as e:
            rospy.logwarn(e)
        return OneArgResponse()

    def unparty(self, req):
        # TODO: Better acknowledgement
        # The default behaviour when speakers are unjoined is to pause
        for speaker in self.speakers_.values():
            speaker.unjoin()
        return EmptyResponse()

    def copy_music(self, req):
        """When given two strings, mirror music played on the first
        on the second"""
        # TODO: Better acknowledgement
        # In SONOS-land, we are joining the destination speaker to the origin
        from_zone = req.zone1
        to_zone = req.zone2
        rospy.loginfo("Copying music from %s to %s", from_zone, to_zone)
        try:
            master_uid = self.speakers_[from_zone].get_speaker_info()["uid"] 
            self.speakers_[to_zone].join(master_uid)
        except KeyError as e:
            rospy.logwarn(e)
        return TwoArgResponse()

    def move_music(self, req):
        """When given two strings, move music played on the first
        to the second"""
        # TODO: Better acknowledgement
        # In SONOS-land, we are joining the destination speaker to the origin,
        # and then removing the origin
        from_zone = req.zone1
        to_zone = req.zone2
        rospy.loginfo("Moving music from %s to %s", from_zone, to_zone)
        self.copy_music(req)
        try:
            self.speakers_[from_zone].unjoin()
        except KeyError as e:
            rospy.logwarn(e)
        return TwoArgResponse()
    
    def play(self, req):
        # TODO: Better acknowledgement
        for speaker in self.speakers_.values():
            speaker.play()
        return EmptyResponse()

    def pause(self, req):
        # TODO: Better acknowledgement
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

