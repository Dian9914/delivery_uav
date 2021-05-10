#!/usr/bin/env python

from delivery_uav.srv import gripper_srv
from geometry_msgs.msg import Wrench
import rospy

pub_r = rospy.Publisher('uav_right_force', Wrench , queue_size=10)
pub_l = rospy.Publisher('uav_left_force', Wrench, queue_size=10)
open_msg = Wrench()
close_msg = Wrench()  

def gripper_handler(req):
    if req.gripper_msg.state:
        print 'Closing the gripper with torque:', req.gripper_msg.torque
        close_msg.torque.y=req.gripper_msg.torque*0.0001
        pub_l.publish(close_msg)
        close_msg.torque.y=-req.gripper_msg.torque*0.0001
        pub_r.publish(close_msg)
    else:
        print("Opening the gripper")
        open_msg.torque.y=-req.gripper_msg.torque*0.0001
        pub_l.publish(open_msg)
        open_msg.torque.y=req.gripper_msg.torque*0.0001
        pub_r.publish(open_msg)
    return True

def gripper_server():
    rospy.init_node('gripper_node')
    s = rospy.Service('gripper_control', gripper_srv, gripper_handler)
    print("Gripper control ready.")
    rospy.spin()

if __name__ == "__main__":
    gripper_server()