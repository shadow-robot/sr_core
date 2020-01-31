#!/usr/bin/env python

# Copyright 2011 Shadow Robot Company Ltd.
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

import rospy
from std_msgs.msg import Float64


class FilterMessages(object):
    def __init__(self):
        rospy.init_node('mean_filter', anonymous=True)

        if rospy.has_param('~nb_occ'):
            nb_occurence = rospy.get_param('~nb_occ')
        else:
            nb_occurence = 5
            rospy.core.logwarn("N number of occurence provided, using 5 as default")

        if rospy.has_param('~in_topic'):
            input_topic_param = rospy.get_param('~in_topic')
            self.subs = rospy.Subscriber(input_topic_param, Float64, self.callback)
            self.pub = rospy.Publisher(input_topic_param+"_filtered", Float64)
        else:
            rospy.core.logerr("No input topic provided, not starting filtering")
            return

        self.queue = []
        self.queue_len = nb_occurence
        self.mean = Float64(0.0)

        rospy.spin()

    def compute_mean(self):
        mean = 0.0
        for val in range(self.queue_len):
            mean = mean+self.queue[val].data

        return mean/self.queue_len

    def callback(self, data):
        self.queue.append(data)

        if len(self.queue) >= self.queue_len:
            self.mean = Float64(self.compute_mean())
            self.queue.pop(0)

        self.pub.publish(self.mean)


if __name__ == '__main__':
    FilterMessages()
