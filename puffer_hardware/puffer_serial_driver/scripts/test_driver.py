#!/usr/bin/env python
import time
import rospy
import random
from puffer_msgs.msg import DifferentialDriveCommand

if __name__ == "__main__":
	rospy.init_node("test_driver")
	rospy.loginfo("Starting the test_driver node")
	pub = rospy.Publisher("vel_cmd", DifferentialDriveCommand, queue_size=1)
	
	cmds = DifferentialDriveCommand()
	rate=rospy.Rate(1)
	
	while not rospy.is_shutdown():
		try:
			cmds.left_motor_command.header.frame_id = "left_wheel"
			cmds.right_motor_command.header.frame_id = "right_wheel"
			text = raw_input("Enter wheel speeds: ").split(",")
			
			if len(text) == 2:
				cmds.left_motor_command.motor_angular_velocity_rad_s = float(text[0])
				cmds.right_motor_command.motor_angular_velocity_rad_s = float(text[1])
			elif len(text) == 1:
				if text[0] == "quit":
					print "---------- Exiting on user input ---------"
					print "---- Stopping the Puffer wheel commands --"
					cmds.left_motor_command.motor_angular_velocity_rad_s = 0
					cmds.right_motor_command.motor_angular_velocity_rad_s = 0
					pub.publish(cmds)
				try:
					speed = float(text[0])
					cmds.left_motor_command.motor_angular_velocity_rad_s = speed
					cmds.right_motor_command.motor_angular_velocity_rad_s = speed
				except:
					print "------------- Not a vlid input ------------"
					print "----- Stopping the Puffer Wheel commands --"
					cmds.left_motor_command.motor_angular_velocity_rad_s = 0
					cmds.right_motor_command.motor_angular_velocity_rad_s = 0
			else:
				print " ---------- Not a vlid input ---------------"
				print "----- Stopping the Puffer Wheel commands --"
				cmds.left_motor_command.motor_angular_velocity_rad_s = 0
				cmds.right_motor_command.motor_angular_velocity_rad_s = 0
		except Exception as e:
			print e
			print " ---------- Exiting on Expcetion ---------------"
			print "------ Stopping the Puffer Wheel commands ------"
			cmds.left_motor_command.motor_angular_velocity_rad_s = 0
			cmds.right_motor_command.motor_angular_velocity_rad_s = 0
		pub.publish(cmds)
		rate.sleep()
