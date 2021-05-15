#!/usr/bin/env python

from delivery_uav.srv import gripper_srv
from geometry_msgs.msg import Wrench
import rospy

class gripper_server():
    def __init__(self):
        self.pub_r = rospy.Publisher('uav_right_force', Wrench , queue_size=10)
        self.pub_l = rospy.Publisher('uav_left_force', Wrench, queue_size=10)
        self.gripper_order = Wrench()

    def gripper_handler(self, req):
        if req.gripper_msg.state=='close':
            print ('GRIPPER NODE: Closing the gripper with torque: %d'%(req.gripper_msg.torque))
            self.gripper_order.torque.y=req.gripper_msg.torque*0.0001
            self.pub_l.publish(self.gripper_order)
            self.gripper_order.torque.y=-req.gripper_msg.torque*0.0001
            self.pub_r.publish(self.gripper_order)
        elif req.gripper_msg.state=='open':
            print("GRIPPER NODE: Opening the gripper")
            self.gripper_order.torque.y=-req.gripper_msg.torque*0.0001
            self.pub_l.publish(self.gripper_order)
            self.gripper_order.torque.y=req.gripper_msg.torque*0.0001
            self.pub_r.publish(self.gripper_order)
        elif req.gripper_msg.state=='idle':
            print("GRIPPER NODE: Setting the gripper in idle state")
            self.gripper_order.torque.y=0
            self.pub_l.publish(self.gripper_order)
            self.gripper_order.torque.y=0
            self.pub_r.publish(self.gripper_order)
        else:
            print("GRIPPER NODE: Incorrect request")
            return False
        return True

    def start_server(self):
        rospy.init_node('gripper_node')
        s = rospy.Service('/del_uav/gripper_node/control', gripper_srv, self.gripper_handler)
        print("GRIPPER NODE: Gripper control ready.")
        rospy.spin()

if __name__ == "__main__":
    server = gripper_server()
    server.start_server()