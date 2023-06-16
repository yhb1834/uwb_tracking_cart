#!/usr/bin/env python3

# This src is adapted from this repo: https://github.com/20chix/dwm1001_ros.git

import rospy
import copy
from interactive_markers.interactive_marker_server  import InteractiveMarkerServer
from interactive_markers.menu_handler               import *
from visualization_msgs.msg                         import (InteractiveMarkerControl, Marker, InteractiveMarker)
from geometry_msgs.msg                              import Point
from geometry_msgs.msg                              import PoseStamped
from uwb_tracking_ros.msg                           import MultiTags


server       = None
rospy.init_node("vizualize_dwm1001")
server = InteractiveMarkerServer("DWM1001_Tags_Server")


anchor1_id = "1781"
anchor2_id = "1782"
anchor3_id = "1783"
tag_id = "A84C"

class VisualizeInRviz:

    def processFeedback(self, feedback):
        """
        Process feedback of markers
        :param: feedback of markers
        :returns: none
        """
        p = feedback.pose.position
        rospy.loginfo(feedback.marker_name + " is pluginsnow at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z))


    def makeBoxControlTag(self,msg):
        """
        Create a box controll for tag
        :param: msg from marker
        :returns: control
        """
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeWhiteSphereTag(msg) )
        msg.controls.append( control )
        return control


    def makeWhiteSphereTag(self, msg ):
        """
        Create a white sphere for tag
        :param: msg from marker
        :returns: marker
        """
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.4
        marker.scale.y = msg.scale * 0.4
        marker.scale.z = msg.scale * 0.4
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 1
        marker.color.a = 1.0
        return marker
    
    def makeAnchorMarker(self, position, name):
        """
        Make coordinates and control for tag
        :param: position of tag
        :param: name for tag
        :returns:
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = name
        int_marker.description = name

        self.makeBoxControlTag(int_marker)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        int_marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        server.insert(int_marker, self.processFeedback)
    
    def makeBox2ControlTag(self,msg):
        """
        Create a box controll for tag
        :param: msg from marker
        :returns: control
        """
        control =  InteractiveMarkerControl()
        control.always_visible = True
        control.markers.append( self.makeRedSphereTag(msg) )
        msg.controls.append( control )
        return control


    def makeRedSphereTag(self, msg ):
        """
        Create a white sphere for tag
        :param: msg from marker
        :returns: marker
        """
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = msg.scale * 0.4
        marker.scale.y = msg.scale * 0.4
        marker.scale.z = msg.scale * 0.4
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1.0
        return marker

    def makeTagMarker(self, position, name):
        """
        Make coordinates and control for tag
        :param: position of tag
        :param: name for tag
        :returns:
        """
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "map"
        int_marker.pose.position = position
        int_marker.scale = 1

        int_marker.name = name
        int_marker.description = name

        self.makeBox2ControlTag(int_marker)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE
        int_marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        server.insert(int_marker, self.processFeedback)


    def AnchorCallback(self,data):
        """
        Callback from topic /dwm1001/tag_id/pose
        :param: data of tag
        :returns:
        """
        global server

        try:
            # Create a new marker with passed coordinates
            position = Point(data.pose.position.x, data.pose.position.y, data.pose.position.z)
            # Add description to the marker
            # self.makeTagMarker(position, "Tag")
            self.makeAnchorMarker(position, data.header.frame_id)
            # Publish marker
            server.applyChanges()

            # Remove this after, Debugging purpose
            # rospy.loginfo("Tag x: " + str(data.pose.position.x) + 
            # " y: " + str(data.pose.position.y) + " z: " + str(data.pose.position.z))

        except ValueError:
           rospy.loginfo("Value error")

    def TagCallback(self,data):
        """
        Callback from topic /dwm1001/tag_id/pose
        :param: data of tag
        :returns:
        """
        global server

        try:
            # Create a new marker with passed coordinates
            position = Point(data.pose.position.x, data.pose.position.y, data.pose.position.z)
            # Add description to the marker
            # self.makeTagMarker(position, "Tag")
            self.makeTagMarker(position, data.header.frame_id)
            # Publish marker
            server.applyChanges()

            # Remove this after, Debugging purpose
            # rospy.loginfo("Tag x: " + str(data.pose.position.x) + 
            # " y: " + str(data.pose.position.y) + " z: " + str(data.pose.position.z))

        except ValueError:
           rospy.loginfo("Value error")
    

    def MultiTagsCallback(self,data):
        """
        Callback from topic /dwm1001/multiTags
        :param: data of tag
        :returns:
        """
        global server

        num_tags = len(data.TagsList)

        for tag in range(num_tags):
            # Process each tags within the TagsList with their corresponding frame ids
            try:
                # Create a new marker with passed coordinates
                position = Point(data.TagsList[tag].pose_x, data.TagsList[tag].pose_y, data.TagsList[tag].pose_z)
                # Add description to the marker
                # self.makeTagMarker(position, "Tag")
                self.makeTagMarker(position, data.TagsList[tag].header.frame_id)
                # Publish marker
                server.applyChanges()

                
            except ValueError:
                rospy.loginfo("Value error")


    def start(self):
        # TODO: auto subscription of multiple tag IDs in a single line (i.e., similar to wildcard)
        rospy.Subscriber("/dwm1001/id_" + anchor1_id + "/pose", PoseStamped, self.AnchorCallback)
        rospy.Subscriber("/dwm1001/id_" + anchor2_id + "/pose", PoseStamped, self.AnchorCallback)
        rospy.Subscriber("/dwm1001/id_" + anchor3_id + "/pose", PoseStamped, self.AnchorCallback)
        rospy.Subscriber("/dwm1001/id_" + tag_id + "/pose", PoseStamped, self.TagCallback)

        #rospy.Subscriber("/dwm1001/multiTags", MultiTags, self.MultiTagsCallback) # multiple tags visualization on RVIZ      
        rospy.spin()


def main():
    DisplayInRviz = VisualizeInRviz()
    DisplayInRviz.start()


if __name__=="__main__":
    main()
