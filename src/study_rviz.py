#!/usr/bin/env python

import rospy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

server = None
menu_handler = MenuHandler()

def point( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.pose.position.x
    marker.scale.y = msg.pose.position.y
    marker.scale.z = msg.pose.position.z
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def pointControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( point(msg) )
    msg.controls.append( control )
    return control


def make6DofMarker(name, oculusdata):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = oculusdata.pose.position
    int_marker.scale = 1

    int_marker.name = name
    int_marker.description = "6-DOF Joint"

    # insert a box
    pointControl(int_marker)
    int_marker.controls[0].interaction_mode = InteractiveMarkerControl.NONE
    

    control = InteractiveMarkerControl()
    control.orientation.w = oculusdata.pose.orientation.w
    control.orientation.x = oculusdata.pose.orientation.x
    control.orientation.y = 0
    control.orientation.z = 0
    #normalizeQuaternion(control.orientation)
    control.name = "rotate_x"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = oculusdata.pose.orientation.w
    control.orientation.x = oculusdata.pose.orientation.x
    control.orientation.y = 0
    control.orientation.z = 0
    #normalizeQuaternion(control.orientation)
    control.name = "move_x"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = oculusdata.pose.orientation.w
    control.orientation.x = 0
    control.orientation.y = oculusdata.pose.orientation.y
    control.orientation.z = 0
    #normalizeQuaternion(control.orientation)
    control.name = "rotate_z"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = oculusdata.pose.orientation.w
    control.orientation.x = 0
    control.orientation.y = oculusdata.pose.orientation.y
    control.orientation.z = 0
    #normalizeQuaternion(control.orientation)
    control.name = "move_z"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = oculusdata.pose.orientation.w
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = oculusdata.pose.orientation.z
    #normalizeQuaternion(control.orientation)
    control.name = "rotate_y"
    control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control)

    control = InteractiveMarkerControl()
    control.orientation.w = oculusdata.pose.orientation.w
    control.orientation.x = 0
    control.orientation.y = 0
    control.orientation.z = oculusdata.pose.orientation.z
    #normalizeQuaternion(control.orientation)
    control.name = "move_y"
    control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )


if __name__=="__main__":
    rospy.init_node("oculus_pose")
    br = TransformBroadcaster()

    # create a timer to update the published transforms
    #rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("oculus_pose")

    menu_handler.insert( "First Entry", callback=processFeedback )
    menu_handler.insert( "Second Entry", callback=processFeedback )
    sub_menu_handle = menu_handler.insert( "Submenu" )
    menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )


    position = Point(0, 0, 0)
    make6DofMarker('left_hand', position, True)





    server.applyChanges()

    rospy.spin()