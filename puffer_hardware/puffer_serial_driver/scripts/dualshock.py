#!/usr/bin/env python
import time
import rospy
import random
from puffer_msgs.msg import DifferentialDriveCommand, PeripheralControl
from sensor_msgs.msg import Joy

class Controller(object):
	""" Class doc """
	
	def __init__ (self):
		""" Class initialiser """
		self.left_wheel 		= rospy.get_param("/dualshock/axes/left_control")
		self.right_wheel 		= rospy.get_param("/dualshock/axes/right_control")
		self.fan_button 		= rospy.get_param("/dualshock/buttons/fan")
		self.e_stop_button 		= rospy.get_param("/dualshock/buttons/e_stop")

		self.drive_commands = DifferentialDriveCommand()
		self.periph_commands = PeripheralControl()
		
		self.max_rad_per_s = rospy.get_param("/puffer/max_vel_per_s") * 2/rospy.get_param("/wheels/diameter")
		self.e_stop_cur = 0
		self.fan_cur = 0
		
	def callback(self,cmd):
		
		a1 = cmd.axes[self.left_wheel]
		a2 = cmd.axes[self.right_wheel]
		b1 = cmd.buttons[self.e_stop_button]
		b2 = cmd.buttons[self.fan_button]
		
		a1 *= self.max_rad_per_s
		a2 *= self.max_rad_per_s

		self.drive_commands.left_motor_command.header.frame_id = "left_wheel"
		self.drive_commands.right_motor_command.header.frame_id = "right_wheel"
		self.drive_commands.left_motor_command.motor_angular_velocity_rad_s = a1
		self.drive_commands.right_motor_command.motor_angular_velocity_rad_s = a2
		
		vel_pub.publish(self.drive_commands)
		
		
		if (b1 and (not self.e_stop_cur)):
			self.periph_commands.e_stop = 1
			self.e_stop_cur = 1
		elif(b1 and self.e_stop_cur):
			self.periph_commands.e_stop = 0
			self.e_stop_cur = 0
		
		if (b2 and (not self.fan_cur)):
			self.periph_commands.fan = 1
			self.fan_cur = 1
		elif(b2 and self.fan_cur):
			self.periph_commands.fan = 0
			self.fan_cur = 0
		
		if (b1 or b2):
			periph_pub.publish(self.periph_commands)
		
		
		
		
		
		
if __name__ == "__main__":
	rospy.init_node("dualshock")
	rospy.loginfo("Starting the dualshock node")
	
	controller = Controller()
	
	sub = rospy.Subscriber("/joy", Joy, controller.callback)
	vel_pub = rospy.Publisher("/vel_cmd", DifferentialDriveCommand, queue_size = 3)
	periph_pub = rospy.Publisher("/peripheral_cmd", PeripheralControl, queue_size = 1)

	rospy.spin()
	
