#!/usr/bin/env python

# ####################################################################
# Copyright (c) 2013 Shadow Robot Company Ltd.
# All rights reserved.
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 2 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
# ####################################################################

import rospy
from controller_manager_msgs.srv import ListControllers, ListControllersResponse
from controller_manager_msgs.msg import ControllerState


class MockHand(object):
    """
    A ros node to look enough like a shadowhand for these tests.

    List controller service so it looks like a ethercat hand to HandCommander.
    """

    def __init__(self, ):
        """
        @brief Construct a new MockHand, setting up ros connections.
        """
        self.list_srv = rospy.Service('controller_manager/list_controllers',
                                      ListControllers, self.list_controllers_cb)
        rospy.loginfo("Started MockHand")

    def list_controllers_cb(self, req):
        cons = ('sr_rh_ffj0_position_controller',
                'sr_rh_ffj3_position_controller',
                'sr_rh_ffj4_position_controller',
                'sr_rh_lfj0_position_controller',
                'sr_rh_lfj3_position_controller',
                'sr_rh_lfj4_position_controller',
                'sr_rh_lfj5_position_controller',
                'sr_rh_mfj0_position_controller',
                'sr_rh_mfj3_position_controller',
                'sr_rh_mfj4_position_controller',
                'sr_rh_rfj0_position_controller',
                'sr_rh_rfj3_position_controller',
                'sr_rh_rfj4_position_controller',
                'sr_rh_thj1_position_controller',
                'sr_rh_thj2_position_controller',
                'sr_rh_thj3_position_controller',
                'sr_rh_thj4_position_controller',
                'sr_rh_thj5_position_controller',
                'sr_rh_wrj1_position_controller',
                'sr_rh_wrj2_position_controller')
        res = ListControllersResponse()
        res.controller = [ControllerState(name=c, state='running') for c in cons]
        return res


if __name__ == "__main__":
    try:
        rospy.init_node("mock_hand")
        node = MockHand()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
