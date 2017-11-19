import rospy
import math
from pid import PID
from yaw_controller import YawController
from std_msgs.msg import Float32

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
	kp = 5.0
	ki = 0.05
	kd = 0.0
	self.pid = PID(kp, ki, kd)
	min_speed = ONE_MPH
	self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
	self.last_time = None
        pass

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	if self.last_time == None:
		self.last_time = rospy.get_time()
		return 0., 0., 0.

	dt = rospy.get_time() - self.last_time
	self.last_time = rospy.get_time()

	if linear_velocity == None or current_velocity == None:
		return 0., 0., 0.

	if dbw_enabled:
		error = linear_velocity.x - current_velocity.twist.linear.x
		throttle = self.pid.step(error, dt)
		#TBD 
		brake = 0.0
		steer = self.yaw_controller.get_steering(linear_velocity.x, angular_velocity.z, current_velocity.twist.linear.x)
        	return throttle, brake, steer
	else:
		self.pid.reset()
		return 0.,0.,0.
