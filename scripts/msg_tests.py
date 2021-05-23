#!/usr/bin/env python

import rospy
from delivery_uav.msg import planner_route, waypoint
from delivery_uav.srv import planner_srv

response = planner_route()

response.point=[[2,3,4,5],[312,213,142]]

goal=waypoint()
goal=[1,2,3]
start = waypoint()
start = [3,2,1]

request = planner_srv._request_class()
request.start=start
request.goal=goal

print response
print response.point[1]
print request