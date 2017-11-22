#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        #sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights
	#rospy.loginfo("traffiic_cb lights:%s", self.lights)
        light_wp, state = self.process_simulator_traffic_lights()
	self.handle_state(light_wp, state)

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
	self.handle_state(light_wp, state)

    def handle_state(self, light_wp, state):
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
	    #rospy.loginfo("Change state to %s, light_wp:%s", self.state, light_wp)
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_square_dist(self, d1, d2):
        x2 = d1.x - d2.x
        y2 = d1.y - d2.y
        return x2*x2 + y2*y2

    def get_square_dist2(self, d1, d2):
        x2 = d1.x - d2[0]
        y2 = d1.y - d2[1]
        return x2*x2 + y2*y2

    def get_closest_waypoint(self, pos):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
	if self.waypoints != None:
        	idx = 0
        	dist = self.get_square_dist(pos, self.waypoints[0].pose.pose.position)
        	for i in range(1, len(self.waypoints)):
                	waypt = self.waypoints[i]
                	d = self.get_square_dist(pos, waypt.pose.pose.position)
                	if (d < dist):
                        	idx = i
                        	dist = d
        	return idx
	else:
		return -1

    def get_closest_waypoint2(self, pos):
	# pos : [x, y]
	#rospy.loginfo("waypoints:%s", self.waypoints)
	if self.waypoints != None:
        	idx = 0
        	dist = self.get_square_dist2(self.waypoints[0].pose.pose.position, pos)
        	for i in range(1, len(self.waypoints)):
                	waypt = self.waypoints[i]
                	d = self.get_square_dist2(waypt.pose.pose.position, pos)
                	if (d < dist):
                        	idx = i
                        	dist = d
        	return idx
	else:
		return -1

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose.position)

        #TODO find the closest visible traffic light (if one exists)

        if light:
            state = self.get_light_state(light)
            return light_wp, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

    def process_simulator_traffic_lights(self):
        """Finds closest visible traffic light from simulator for testing only

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
	light_wp = -1
	stop_line_pos = None
        if (self.pose and stop_line_positions):
	    #rospy.loginfo("cur pos:%s, stop:%s", self.pose.pose.position, stop_line_positions)
	    idx = 0
            dist = self.get_square_dist2(self.pose.pose.position, stop_line_positions[0])
            for i in range(1, len(stop_line_positions)):
               	d = self.get_square_dist2(self.pose.pose.position, stop_line_positions[i])
                if (d < dist):
                    dist = d
		    idx = i
	    stop_line_pos = stop_line_positions[idx]
            light_wp = self.get_closest_waypoint2(stop_line_pos)
	    #rospy.loginfo("cur pos:%s, stop:%s, light_wp:%s", self.pose.pose.position, stop_line_pos, light_wp)

        #TODO find the closest visible traffic light (if one exists)
	state = TrafficLight.UNKNOWN
	if self.lights != None and stop_line_pos != None:
	    state = self.lights[0].state
            dist = self.get_square_dist2(self.lights[0].pose.pose.position, stop_line_pos)
	    for i in range(1,len(self.lights)):
               	d = self.get_square_dist2(self.lights[i].pose.pose.position, stop_line_pos)
                if (d < dist):
                    dist = d
		    state = self.lights[i].state
		    break;

	
        return light_wp, state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
