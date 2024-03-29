#!/usr/bin/python3

# Copyright 2014-2022, 2023 Shadow Robot Company Ltd.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation version 2 of the License.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program. If not, see <http://www.gnu.org/licenses/>.

from __future__ import absolute_import
import rospy
import rostopic
from sr_robot_msgs.msg import BiotacAll, ShadowPST, UBI0All


class TactileReceiver:

    """
    Module that receives the tactile values.
    """

    def __init__(self, prefix=""):
        """
        Receives the tactile information from a Shadow Hand
        @param prefix - prefix for the tactile topic
        """
        # appends trailing slash if necessary
        if not prefix.endswith("/"):
            # kind of a hack to remove trailing underscores
            if prefix.endswith("_"):
                prefix = prefix[:-1]
            prefix += "/"

        self._prefix = prefix

        self.tactile_type = self.find_tactile_type()
        self.tactile_state = None

        if self.tactile_type == "PST":
            self.tactile_listener = rospy.Subscriber(
                prefix + "tactile", ShadowPST, self.tactile_callback, queue_size=1)
        elif self.tactile_type == "biotac":
            self.tactile_listener = rospy.Subscriber(
                prefix + "tactile", BiotacAll, self.tactile_callback, queue_size=1)
        elif self.tactile_type == "UBI0":
            self.tactile_listener = rospy.Subscriber(
                prefix + "tactile", UBI0All, self.tactile_callback, queue_size=1)

    def find_tactile_type(self):
        message_type = rostopic.get_topic_type("/" + self._prefix + "tactile")[0]
        if message_type:
            if "ShadowPST" in message_type:
                return "PST"
            if "BiotacAll" in message_type:
                return "biotac"
            if "UBI0All" in message_type:
                return "UBI0"

        rospy.logwarn("No supported tactile topic found. This is normal for a simulated hand")
        return None

    def tactile_callback(self, tactile_msg):
        self.tactile_state = tactile_msg

    def get_tactile_type(self):
        return self.tactile_type

    def get_tactile_state(self):
        return self.tactile_state
