#!/usr/bin/env python

import rospy
from delivery_uav.msg import planner_route, waypoint
from delivery_uav.srv import planner_srv, user_interface

response = planner_srv._request_class()
request = user_interface._request_class()

response.goal.xyz=[1,2,3]


print response
print response.goal.xyz[1]


print request
print request.user_cmd.goal.xyz