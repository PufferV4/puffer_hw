#!/usr/bin/env python
from puffer_msgs.msg import DifferentialDriveCommand, DifferentialDriveEncoders, MotorControllerStatus, PeripheralControl, Battery
import time
import rospy
from puffer_serial_driver import SerialDriver

def command_callback(cmds):
	puffer.update_cmd_buf(
			cmds.left_motor_command.motor_angular_velocity_rad_s, 
			cmds.right_motor_command.motor_angular_velocity_rad_s
			)

def periph_callback(data):
	puffer.set_estop(data.e_stop)
	#puffer.enable_12v_reg(data.fan)
	
def shutdown():
	rospy.loginfo("puffer node shutting down")
	puffer.cleanup()
	
if __name__ == "__main__":
	puffer = SerialDriver()
	rospy.init_node("puffer")
	rospy.loginfo("Starting the puffer node")
	
	rospy.on_shutdown(shutdown)
	
	sub = rospy.Subscriber("vel_cmd", DifferentialDriveCommand, command_callback)
	sub2 = rospy.Subscriber("/peripheral_cmd", PeripheralControl, periph_callback)
	enc_pub = rospy.Publisher("encoders", DifferentialDriveEncoders, queue_size=1)
	state_pub = rospy.Publisher("puffer_state", MotorControllerStatus, queue_size=1)
	battery_pub = rospy.Publisher("/battery_info", Battery, queue_size=1)
	
	enc = DifferentialDriveEncoders()
	status = MotorControllerStatus()
	batt = Battery()
	rate=rospy.Rate(rospy.get_param("/puffer/rate"))
	counter = 0
	while not rospy.is_shutdown():
		puffer.loop(counter)
		
		enc.left_motor_encoder.angular_velocity_rad_s = puffer.enc[0]
		enc.right_motor_encoder.angular_velocity_rad_s = puffer.enc[1]
		now = rospy.Time.now()									#values incriment in seconds, think about changing to more accurate method
		enc.left_motor_encoder.angular_position_measurement_time = puffer.timestamp
		enc.right_motor_encoder.angular_position_measurement_time = puffer.timestamp
		enc.left_motor_encoder.header.frame_id = "left_wheel"
		enc.right_motor_encoder.header.frame_id = "right_wheel"
		enc.right_motor_encoder.header.stamp = now
		enc.left_motor_encoder.header.stamp = now
		enc.right_motor_encoder.header.seq = counter
		enc.left_motor_encoder.header.seq = counter
		
		status.voltage = puffer.voltage
		status.temperature = puffer.temp
		status.roboclaw_error = puffer.error
		status.left_motor_current = puffer.currents[0]
		status.right_motor_current = puffer.currents[1]
		status.max_left_integrator = puffer.max_left_integrator
		status.max_right_integrator = puffer.max_right_integrator
		status.left_motor_integrator = puffer.left_integrator
		status.right_motor_integrator = puffer.right_integrator
		status.e_stop_state = puffer.e_stop
		status.reg_state = puffer.reg_enabled

		batt.voltage = puffer.voltage
		batt.percent_charged = puffer.battery_percent
		batt.shutdown_warning = puffer.shutdown_warning
		batt.shutdown_flag = puffer.shutdown_flag

		enc_pub.publish(enc)
		state_pub.publish(status)
		battery_pub.publish(batt)

		counter +=1
		rate.sleep()
