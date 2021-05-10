#!/usr/bin/env python

from delivery_uav.srv import gripper_srv, user_interface
import rospy

def controller_handler(req):
    return True

def controller_server():
    rospy.init_node('controller_node')
    s = rospy.Service('uav_user_interface', user_interface, controller_handler)
    print("CONTROLLER NODE: Controller node ready.")
    rospy.spin()

if __name__ == "__main__":
    controller_server()