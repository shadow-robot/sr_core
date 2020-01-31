#!/usr/bin/env python

# Copyright 2017 Shadow Robot Company Ltd.
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


class SaveUrdfAndSrdf(object):
    def __init__(self):
        pass

    def save_urdf_from_param(self, path_to_save_files="/tmp", file_name="robot"):
        if rospy.has_param('/robot_description'):
            urdf_str = rospy.get_param('/robot_description')
            path = path_to_save_files + "/" + file_name + ".urdf"
            urdf_file = open(path, "wb")
            urdf_file.write(urdf_str)
            urdf_file.close()
            return path
        else:
            rospy.logerr("Parameter server does not have /robot_description param")

    def save_srdf_from_param(self, path_to_save_files="/tmp", file_name="robot"):
        if rospy.has_param('/robot_description_semantic'):
            srdf_str = rospy.get_param('/robot_description_semantic')
            path = path_to_save_files + "/" + file_name + ".srdf"
            srdf_file = open(path, "wb")
            srdf_file.write(srdf_str)
            srdf_file.close()
            return path
        else:
            rospy.logerr("Parameter server does not have /robot_description_semantics param")


if __name__ == '__main__':
    rospy.init_node("saving_urdf_and_srdf_to_files", anonymous=True)
    save_files = SaveUrdfAndSrdf()
    save_files.save_urdf_from_param()
    save_files.save_srdf_from_param()
