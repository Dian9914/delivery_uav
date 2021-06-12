#!/usr/bin/env python

import rospy
from delivery_uav.msg import planner_route, waypoint
from delivery_uav.srv import planner_srv, user_interface

response = planner_srv._response_class()
request = user_interface._request_class()




print response
print response.route.point

print request
print request.user_cmd.goal.xyz