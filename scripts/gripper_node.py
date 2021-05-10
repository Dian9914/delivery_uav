#!/usr/bin/env python

from delivery_uav.srv import gripper_srv
from geometry_msgs.msg import Wrench
import rospy

pub_r = rospy.Publisher('uav_right_force', Wrench , queue_size=10)
pub_l = rospy.Publisher('uav_left_force', Wrench, queue_size=10)
gripper_order = Wrench()

def gripper_handler(req):
    if req.gripper_msg.state=='close':
        print 'GRIPPER NODE: Closing the gripper with torque:', req.gripper_msg.torque
        gripper_order.torque.y=req.gripper_msg.torque*0.0001
        pub_l.publish(gripper_order)
        gripper_order.torque.y=-req.gripper_msg.torque*0.0001
        pub_r.publish(gripper_order)
    elif req.gripper_msg.state=='open':
        print("GRIPPER NODE: Opening the gripper")
        gripper_order.torque.y=-req.gripper_msg.torque*0.0001
        pub_l.publish(gripper_order)
        gripper_order.torque.y=req.gripper_msg.torque*0.0001
        pub_r.publish(gripper_order)
    elif req.gripper_msg.state=='neutral':
        print("GRIPPER NODE: Setting the gripper in neutral state")
        gripper_order.torque.y=0
        pub_l.publish(gripper_order)
        gripper_order.torque.y=0
        pub_r.publish(gripper_order)
    else:
        print("GRIPPER NODE: Incorrect request")
        return False
    return True

def gripper_server():
    rospy.init_node('gripper_node')
    s = rospy.Service('gripper_control', gripper_srv, gripper_handler)
    print("GRIPPER NODE: Gripper control ready.")
    rospy.spin()

if __name__ == "__main__":
    gripper_server()