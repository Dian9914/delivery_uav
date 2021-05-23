#!/usr/bin/env python

import rospy
from delivery_uav.msg import planner_route

response = planner_route()

response.point=[[2,3,4,5],[312,213,142]]

print response
print response.point[1]