import rospy
import math
from pid import PID
from yaw_controller import YawController
from std_msgs.msg import Float32

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, wheel_base, steer_ratio, max_lat_accel, max_steer_angle, wheel_radius, vehicle_mass, brake_deadband, decel_limit, accel_limit):
        # TODO: Implement
	self.wheel_radius = wheel_radius
	self.vehicle_mass = vehicle_mass
	self.brake_deadband = brake_deadband
	self.decel_limit = decel_limit
	self.accel_limit = accel_limit
	kp = 5.0
	ki = 0.05
	kd = 0.01
	self.pid = PID(kp, ki, kd)
	min_speed = ONE_MPH
	self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
	self.last_time = None
	self.last_velocity = None

    def control(self, linear_velocity, angular_velocity, current_velocity, dbw_enabled, fuel_amount):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
	if self.last_time == None or current_velocity == None:
		self.last_time = rospy.get_time()
		return 0., 0., 0.

	dt = rospy.get_time() - self.last_time
	self.last_time = rospy.get_time()
        last_vel = self.last_velocity
	self.last_velocity = current_velocity.twist.linear.x

	if linear_velocity == None or current_velocity == None:
		return 0., 0., 0.

	if dbw_enabled:
		error = linear_velocity.x - current_velocity.twist.linear.x
		throttle = self.pid.step(error, dt)
		throttle = max(min(throttle, 1.0), 0.0)
		# calc accel & brake
		brake = 0
		if error < 0:
			accel = -error / dt
			accel = min(max(accel, self.decel_limit), self.accel_limit) 
			brake = (self.vehicle_mass + fuel_amount * GAS_DENSITY) * accel * self.wheel_radius
			if brake < self.brake_deadband:
				brake = 0
			throttle = 0
			#rospy.loginfo(" error:%s, throttle:%s, brake:%s, accel:%s, self.brake_deadband:%s", error, throttle, brake, accel, self.brake_deadband);
		else:
			brake = 0
		steer = self.yaw_controller.get_steering(linear_velocity.x, angular_velocity.z, current_velocity.twist.linear.x)
        	return throttle, brake, steer
	else:
		self.pid.reset()
		return 0.,0.,0.
