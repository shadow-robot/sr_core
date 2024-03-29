#!/usr/bin/env python3

# Copyright 2011, 2022, 2023 Shadow Robot Company Ltd.
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

from interactive_markers.interactive_marker_server import InteractiveMarker, InteractiveMarkerFeedback
from visualization_msgs.msg import Marker, InteractiveMarkerControl


class InteractiveConnectorSelector:
    def __init__(self, object_names, callback_fct, interactive_server):
        # create an interactive marker server on the topic namespace simple_marker
        self.server = interactive_server

        self.callback_function = callback_fct
        self.object_names = object_names

        self.int_markers = {}
        self.object_markers = {}
        self.object_controls = {}
        self.select_controls = {}

        for object_name in self.object_names:
            self.create_marker(object_name)

    def create_marker(self, object_name):
        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "/"+object_name
        int_marker.name = object_name
        int_marker.description = "Select this object"
        int_marker.pose.position.z = -0.3

        self.int_markers[object_name] = int_marker

        # create a marker over the object
        object_marker = Marker()
        object_marker.type = Marker.SPHERE
        object_marker.pose.position.z = 0.6

        object_marker.scale.x = 0.1
        object_marker.scale.y = 0.1
        object_marker.scale.z = 0.1
        object_marker.color.r = 0.6
        object_marker.color.g = 0.02
        object_marker.color.b = 1.0
        object_marker.color.a = 1.0
        self.object_markers[object_name] = object_marker

        # create a non-interactive control which contains the box
        object_control = InteractiveMarkerControl()
        object_control.interaction_mode = InteractiveMarkerControl.BUTTON
        object_control.always_visible = True
        object_control.markers.append(object_marker)

        self.object_controls[object_name] = object_control

        # add the control to the interactive marker
        self.int_markers[object_name].controls.append(object_control)

        # add the interactive marker to our collection &
        # tell the server to call process_feedback() when feedback arrives for it
        self.server.insert(self.int_markers[object_name], self.process_feedback)

        # 'commit' changes and send to all clients
        self.server.applyChanges()

    def process_feedback(self, feedback):
        # this was not a click on the sphere
        if feedback.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        # One of the object has been selected.
        # We'll update all the dynamic markers
        # to make the selected one prominent.

        # this is the name of the selected object.
        selected_name = feedback.marker_name

        # we loop through all our interactive markers.
        for name in self.object_controls.keys():  # pylint: disable=C0206
            self.object_controls[name].markers.remove(self.object_markers[name])

            if name == selected_name:
                # the selected object marker is set
                # higher, bigger and in green
                self.object_markers[name].pose.position.z = 0.65

                self.object_markers[name].scale.x = 0.15
                self.object_markers[name].scale.y = 0.15
                self.object_markers[name].scale.z = 0.15
                self.object_markers[name].color.r = 0.32
                self.object_markers[name].color.g = 0.83
                self.object_markers[name].color.b = 0.15
            else:
                # the other objects are slightly lower,
                # smaller and in purple
                self.object_markers[name].pose.position.z = 0.6

                self.object_markers[name].scale.x = 0.1
                self.object_markers[name].scale.y = 0.1
                self.object_markers[name].scale.z = 0.1
                self.object_markers[name].color.r = 0.6
                self.object_markers[name].color.g = 0.02
                self.object_markers[name].color.b = 1.0

            self.object_controls[name].markers.append(self.object_markers[name])

            self.server.insert(self.int_markers[name])

        # we update the interactive marker server
        self.server.applyChanges()
        self.callback_function(selected_name)
