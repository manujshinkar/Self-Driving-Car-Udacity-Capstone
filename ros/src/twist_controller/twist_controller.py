import rospy
from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
	def __init__(self, car_params):
		self.car_params = car_params

		self.lpf_accel = LowPassFilter(tau = 0.5, ts = 0.02)

		self.accel_pid = PID(kp = 0.4, ki = 0.1, kd = 0.0, mn = 0.0, mx = 1.0)
		self.speed_pid = PID(kp = 2.0, ki = 0.0, kd = 0.0, mn = car_params.decel_limit,
							 mx = car_params.accel_limit)

		self.yaw_control = YawController(car_params.wheel_base, car_params.steer_ratio, 4. * ONE_MPH, car_params.max_lat_accel, car_params.max_steer_angle)
		
		self.prev_velocity = 0

	def control(self, proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity, dbw_enabled, control_period):
		# TODO: Change the arg, kwarg list to suit your needs
		
		accel = (current_linear_velocity - self.prev_velocity) / control_period
		self.lpf_accel.filt(accel)
		self.prev_velocity = current_linear_velocity

		vel_error = proposed_linear_velocity - current_linear_velocity

		if abs(proposed_linear_velocity) < ONE_MPH:
			self.speed_pid.reset()

		accel_cmd = self.speed_pid.step(vel_error, control_period)

		min_speed = ONE_MPH * 5
		if proposed_linear_velocity < 0.01:
			accel_cmd = min(accel_cmd,
							-530. / self.car_params.vehicle_mass / self.car_params.wheel_radius)
		elif proposed_linear_velocity < min_speed:
			proposed_angular_velocity *= min_speed/proposed_linear_velocity
			proposed_linear_velocity = min_speed


		throttle = brake = steer = 0
		if dbw_enabled:		
			if accel_cmd >= 0:
				throttle = self.accel_pid.step(accel_cmd - self.lpf_accel.get(),control_period)
			else:
				self.accel_pid.reset()

			if (accel_cmd < -self.car_params.brake_deadband) or (proposed_linear_velocity < min_speed):
				brake = -accel_cmd * self.car_params.vehicle_mass * self.car_params.wheel_radius

			steer = self.yaw_control.get_steering(proposed_linear_velocity, proposed_angular_velocity, current_linear_velocity)
			rospy.loginfo('%s,%s,%s',throttle, brake, steer);
		else :
			self.accel_pid.reset()
			self.speed_pid.reset()

		# Return throttle, brake, steer
		return throttle, brake, steer
