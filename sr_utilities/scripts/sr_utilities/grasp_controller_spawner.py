#!/usr/bin/env python
#
# Copyright 2015 Shadow Robot Company Ltd.
#
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
#
import rospy
import yaml
import rospkg
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController, LoadController
from std_msgs.msg import Bool


class GraspControllerSpawner(object):
    def __init__(self, service_timeout):
        self.service_timeout = service_timeout

    def set_controller(self):
        controller_to_start = 'sh_rh_grasp_controller'
        success = True

        try:
            rospy.wait_for_service('controller_manager/load_controller', self.service_timeout)
            load_controllers = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
            loaded_controllers = load_controllers(controller_to_start)
        except rospy.ServiceException:
            success = False
        if not loaded_controllers.ok:
            success = False

        try:
            rospy.wait_for_service('controller_manager/switch_controller', self.service_timeout)
            switch_controllers = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)
            switched_controllers = switch_controllers([controller_to_start], None,
                                                      SwitchController._request_class.BEST_EFFORT)
        except rospy.ServiceException:
            success = False

        if not switched_controllers.ok:
            success = False

        if not success:
            rospy.logerr("Failed to launch grasp controller!")

    @staticmethod
    def wait_for_topic(topic_name, timeout):
        if not topic_name:
            return True

        # This has to be a list since Python has a peculiar mechanism to determine
        # whether a variable is local to a function or not:
        # if the variable is assigned in the body of the function, then it is
        # assumed to be local. Modifying a mutable object (like a list)
        # works around this debatable "design choice".
        wait_for_topic_result = [None]

        def wait_for_topic_cb(msg):
            wait_for_topic_result[0] = msg
            rospy.logdebug("Heard from wait-for topic: %s" % str(msg.data))
        rospy.Subscriber(topic_name, Bool, wait_for_topic_cb)
        started_waiting = rospy.Time.now().to_sec()

        # We might not have received any time messages yet
        warned_about_not_hearing_anything = False
        while not wait_for_topic_result[0]:
            rospy.sleep(0.01)
            if rospy.is_shutdown():
                return False
            if not warned_about_not_hearing_anything:
                if rospy.Time.now().to_sec() - started_waiting > timeout:
                    warned_about_not_hearing_anything = True
                    rospy.logwarn("Controller Spawner hasn't heard anything from its \"wait for\" topic (%s)" %
                                  topic_name)
        while not wait_for_topic_result[0].data:
            rospy.sleep(0.01)
            if rospy.is_shutdown():
                return False
        return True


if __name__ == "__main__":
    rospy.init_node("grasp_controller_spawner")
    wait_for_topic = rospy.get_param("~wait_for", "")
    timeout = rospy.get_param("~timeout", 30.0)
    service_timeout = rospy.get_param("~service_timeout", 60.0)

    grasp_spawner = GraspControllerSpawner(service_timeout=service_timeout)
    if grasp_spawner.wait_for_topic(wait_for_topic, timeout):
        grasp_spawner.set_controller()
