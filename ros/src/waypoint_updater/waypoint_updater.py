#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from std_msgs .msg import Int32

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
SLOWDOWN_WPS = 10 # Number of waypoints starting to slow down to stop
MPH2MPS = 0.44704
SPEED = 15 * MPH2MPS

# utility to calculate square distance between 2 points 
def get_square_dist(d1, d2):
	x2 = d1.x - d2.x
	y2 = d1.y - d2.y
	return x2*x2 + y2*y2

# get the closest waypoint to pos
def get_closest_waypoint(pos, waypoints):
	idx = 0
	dist = get_square_dist(pos, waypoints[0].pose.pose.position)
	for i in range(1, len(waypoints)):
		waypt = waypoints[i]
		d = get_square_dist(pos, waypt.pose.pose.position)
		if (d < dist):
			idx = i
			dist = d
	return idx

# get next waypoints starting from idx up to count
def get_next_waypoints(waypoints, idx, count):
	last = min(len(waypoints), idx+count)
	return [waypoints[i] for i in range(idx+1, last)]

# get lane from the waypoints with given id
def get_lane_object(id, waypoints):
	lane = Lane()
	lane.header.frame_id = id
	lane.header.stamp = rospy.Time.now()
	lane.waypoints = waypoints
	return lane

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # current position of the car
	self.cur_pos = None

	# list of base waypoints
	self.base_waypoints = None

	# traffic waypoint index
	self.traffic_waypt_index = None

        rospy.spin()


    def pose_cb(self, msg):
        # TODO: Implement
        self.cur_pos = msg.pose.position
	frame_id = msg.header.frame_id

	if self.base_waypoints != None:
		# get the closet waypont to the car
		idx = get_closest_waypoint(self.cur_pos, self.base_waypoints)
		# get look ahead waypoints
		points = get_next_waypoints(self.base_waypoints, idx, LOOKAHEAD_WPS)
		rospy.loginfo("pos_cb: %s, points:%s, cur_pos:%s", idx, len(points), self.cur_pos)
		slowdown_index = None
		delta = 0
		if self.traffic_waypt_index != None and self.traffic_waypt_index >= idx:
			slowdown_index = max(self.traffic_waypt_index - SLOWDOWN_WPS, idx)
			count = max(self.traffic_waypt_index - slowdown_index - 2, 0)
			if count != 0:
				delta = SPEED / count
			else:
				delta = SPEED
			slowdown_end = slowdown_index + count 
			rospy.loginfo("idx: %s, traffic_idx:%s, slowdown_idx:%s, end:%s, delta:%s, count:%s", idx, self.traffic_waypt_index, slowdown_index, slowdown_end, delta, count)
		# set speed
		i = 0
		slowness = SPEED
		for pt in points:
			if slowdown_index != None :
				if i < slowdown_index or i > self.traffic_waypt_index:
					pt.twist.twist.linear.x = SPEED
				else:
					pt.twist.twist.linear.x = 0	
				#elif i >=  slowdown_index and i < slowdown_end:
				#	pt.twist.twist.linear.x = slowness
				#	slowness -= delta
				#elif i <  self.traffic_waypt_index + 1:
				#	pt.twist.twist.linear.x = 0	
				#	slowness = 0
				#else:
				#	slowness += SPEED / 10
				#	pt.twist.twist.linear.x = max(SPEED, slowness)
				i += 1
			else:
				pt.twist.twist.linear.x = SPEED
			#rospy.loginfo("  x:%s, y:%s", pt.pose.pose.position.x, pt.pose.pose.position.y)
		# make lane object
		lane = get_lane_object(frame_id, points)
		# publish  lane
		self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypt_index = msg.data
	rospy.loginfo("traffic waypt_index:%s", msg.data)
	if (self.traffic_waypt_index < 0):
		self.traffic_waypt_index = None

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
