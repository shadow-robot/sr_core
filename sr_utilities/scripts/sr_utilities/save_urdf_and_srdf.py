#!/usr/bin/env python3

# Copyright 2017, 2022 Shadow Robot Company Ltd.
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


class SaveUrdfAndSrdf:
    def __init__(self):
        pass

    @staticmethod
    def save_urdf_from_param(path_to_save_files="/tmp", file_name="robot"):
        if rospy.has_param('/robot_description'):
            urdf_str = rospy.get_param('/robot_description')
            path = path_to_save_files + "/" + file_name + ".urdf"
            with open(path, "w", encoding="utf-8") as urdf_file:
                urdf_file.write(urdf_str)
            print("urdf saved successfully:" + path)
            return path
        rospy.logerr("Parameter server does not have /robot_description param")
        return None

    @staticmethod
    def save_srdf_from_param(path_to_save_files="/tmp", file_name="robot"):
        if rospy.has_param('/robot_description_semantic'):
            srdf_str = rospy.get_param('/robot_description_semantic')
            path = path_to_save_files + "/" + file_name + ".srdf"
            with open(path, "w", encoding="utf-8") as srdf_file:
                srdf_file.write(srdf_str)
            print("srdf saved successfully:" + path)
            return path
        rospy.logerr("Parameter server does not have /robot_description_semantics param")
        return None


if __name__ == '__main__':
    rospy.init_node("saving_urdf_and_srdf_to_files", anonymous=True)
    save_files = SaveUrdfAndSrdf()
    save_files.save_urdf_from_param()
    save_files.save_srdf_from_param()
