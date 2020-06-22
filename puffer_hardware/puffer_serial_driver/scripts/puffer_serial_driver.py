#!/usr/bin/env python
from roboclaw import Roboclaw
import time
from datetime import datetime
import serial
import math
import rospy
import Jetson.GPIO as GPIO


class SerialDriver(object):
	'''
	Class for serial UART interface to the RoboClaw Motor Controllers
	
	'''
	def __init__(self):
		rospy.loginfo("Initilizing the motor controllers..")

		self.e_stop 			= 1
		self.reg_enabled 		= 0
		self.temp 			= 0
		self.error 			= 0
		self.voltage 			= 0

		self.currents 			= [0,0]
		self._thread_lock 		= False
		
		self.prev_enc_ts 		= None
		self.prev_tick 			= [None, None]

		self.start_time			= datetime.now()
		self.left_currents		= []
		self.right_currents		= []
		self.max_left_integrator	= 0
		self.max_right_integrator 	= 0
		self.left_integrator		= 0
		self.right_integrator		= 0
		self.motor_lockout 		= 0

		self._cmd_buf_flag		= 0
		self._l_vel_cmd_buf		= 0
		self._r_vel_cmd_buf		= 0
		self.battery_percent 		= 0
		self.shutdown_warning		= False
		self.shutdown_flag		= False

		self.rate = rospy.get_param("/puffer/rate")
		self.delta_t = 2.0/self.rate
		self.rc = Roboclaw(
				rospy.get_param("/motor_controllers/serial_device_name"), 
				rospy.get_param("/motor_controllers/baud_rate")
				)

		self.rc.Open()
		self._set_operating_params()
		self._get_version()
		self.set_estop(0)					#Clears E-stop pin
		self.enable_12v_reg(1)					#Disables 12V Regulator
		self.kill_motors()					#Start the motors not moving
		
	#Private Methods
	def _get_version(self):
		'''
		Version check for communication verification to the motor controllers
		
		returns ther version number if sucessful, and 0 if not
		'''
		
		
		version = self.rc.ReadVersion(self.address)
		if version != 0:
			rospy.loginfo("[Motor __init__ ] Sucessfully connected to all Motor Controllers!")
			rospy.loginfo( version )
			rospy.set_param("/motor_controllers/firmware_version",version)
		else:
			raise Exception("Unable to establish connection to Motor controllers")
		return version

	def _set_operating_params(self):
		'''
		Sets all the operating parameters for control of PUFFER, Pinouts, and 
		safety parameters.
		
		returns None
		'''

		#GPIO settings for E-stop and Regulator Enable pins
		self.e_stop_pin = rospy.get_param("/gpio_pins/e_stop",1)
		self.reg_en_pin = rospy.get_param("/gpio_pins/reg_en",1)
		
		try:
			GPIO.setmode(GPIO.BCM)
			GPIO.setwarnings(False)
			GPIO.setup(self.e_stop_pin, GPIO.OUT)
			GPIO.setup(self.reg_en_pin, GPIO.OUT)
		except:
			pass
		#Threadlock used for serial comm to RoboClaw
		self.thread_timeout		= rospy.get_param("/threadlock/timeout")

		#Motor operating parameters
		self.wheel_d = rospy.get_param("/wheels/diameter")
		self.enc_cpr = rospy.get_param("/wheels/encoder/cts_per_rev")
		factor = rospy.get_param("/wheels/encoder/factor")
		stage_1 = rospy.get_param("/wheels/gearbox/stage1")
		stage_2 = rospy.get_param("/wheels/gearbox/stage2")
		stage_3 = rospy.get_param("/wheels/gearbox/stage3")
		
		self.accel_const = rospy.get_param("/puffer/accel_const")
		self.max_vel_per_s = rospy.get_param("/puffer/max_vel_per_s")
		self.tick_per_rev = int(self.enc_cpr * factor * stage_1 * stage_2 * stage_3)
		rospy.loginfo(self.tick_per_rev)
		rospy.set_param("tick_per_rev", self.tick_per_rev)

		self.max_cts_per_s = int((self.max_vel_per_s * self.tick_per_rev)/(math.pi * self.wheel_d))
		self.max_accel = int(self.max_cts_per_s * self.accel_const)
		
		rospy.set_param("max_cts_per_s", self.max_cts_per_s)
		rospy.set_param("max_accel", self.max_accel)
		self.address = rospy.get_param("/motor_controllers/address", 0x80)
		self.rc.SetMainVoltages(self.address,int(rospy.get_param("/motor_controllers/battery/main/low", 12.0) * 10),int(rospy.get_param("motor_controllers/battery/main/high", 18.0 ) * 10))
		self.rc.SetLogicVoltages(self.address,int(rospy.get_param("/motor_controllers/battery/logic/low") * 10),int(rospy.get_param("motor_controllers/battery/logic/high") * 10))
		
		self.max_current = rospy.get_param("/motor_controllers/current/max_amps")
		self.motor_lockout_time = rospy.get_param("/puffer/motor_lockout_time")
		
		m1p = rospy.get_param("/motor_controllers/m1/p")
		m1i = rospy.get_param("/motor_controllers/m1/i")
		m1d = rospy.get_param("/motor_controllers/m1/d")
		m1qpps = rospy.get_param("/motor_controllers/m1/qpps")
		m2p = rospy.get_param("/motor_controllers/m2/p")
		m2i = rospy.get_param("/motor_controllers/m2/i")
		m2d = rospy.get_param("/motor_controllers/m2/d")
		m2qpps = rospy.get_param("/motor_controllers/m2/qpps")
		

		self.battery_max_time = rospy.get_param("/battery/max_time")
		self.battery_max_volts = rospy.get_param("/battery/max_volts")
		self.battery_coef_a = rospy.get_param("/battery/a")
		self.battery_coef_b = rospy.get_param("/battery/b")
		self.battery_coef_c = rospy.get_param("/battery/c")
		self.battery_warning = rospy.get_param("/battery/warning_percent")
		self.battery_shutdown = rospy.get_param("/battery/shutdown_percent")

		self.rc.SetM1VelocityPID(self.address,m1p, m1i, m1d, m1qpps)
		self.rc.SetM2VelocityPID(self.address,m2p, m2i, m2d, m2qpps)
		
		self.rc.WriteNVM(self.address)
		time.sleep(0.001)
		self.rc.ReadNVM(self.address)
		
	def _lock_thread(self,lock):
		'''
		Checks the thread lock and then grabs it when it frees up

		no return value

		'''
		if (lock):
			start = time.time()
			while self._thread_lock:
				#rospy.loginfo("in threadlock")
				if time.time() - start > self.thread_timeout:
					raise Exception("Thread lock timeout")
				time.sleep(0.001)
			self._thread_lock = True
		else:
			self._thread_lock = False

	def _get_Temp(self):
		'''
		Gets the temperature of the motor controllers
		
		return:
		list [2] (int): Temperature values * 10 degrees C
		
		'''
		self._lock_thread(1)
		self.temp = self.rc.ReadTemp(self.address)[1]
		self._lock_thread(0)
		self.temp = int(self.temp*100)/1000.0
		
		return self.temp	

	def _get_Voltage(self):
		'''
		Gets the voltage of the motor controllers
		
		return:
		voltage (int) : Voltage values * 10 volts
		
		'''
		self._lock_thread(1)
		v = self.rc.ReadMainBatteryVoltage(self.address)[1]
		v = int(v*100)/1000.0
		if v != 0:
			self.voltage = v + 0.4     #accounts for the voltage drop in the diode
		self._lock_thread(0)
		return v

	def _get_Currents(self):
		'''
		Gets the current of the motor controllers
		
		return:
		list [2] (int): Current values * 100 Amps
		
		'''
		self._lock_thread(1)
		cur = self.rc.ReadCurrents(self.address)
		self._lock_thread(0)
		r_current = int(cur[1])/100.0
		l_current = int(cur[2])/100.0
		self.currents = [l_current,r_current]
		
		self.left_currents.insert(0, l_current)
		self.right_currents.insert(0, r_current)

		if (len(self.left_currents) > self.rate/2):
			del self.left_currents[-1]
			del self.right_currents[-1]

		left_power = 0
		right_power = 0

		for i in range(len(self.left_currents)):
			left_power += math.pow(self.left_currents[i],2) * self.delta_t
			right_power += math.pow(self.right_currents[i],2) * self.delta_t

		if (left_power >= math.pow(self.max_current,2) or right_power >= math.pow(self.max_current,2)):
			rospy.loginfo("Motor power exceeded max allowed! Disabling motors for %d seconds" %(self.motor_lockout_time))
			self.motor_lockout = 1
			self.set_estop(1)
			self.lockout_timestamp = time.time()
			
		if (self.motor_lockout and (time.time() - self.lockout_timestamp >= self.motor_lockout_time)):
			rospy.loginfo("Re-enabling the motors from timeout lock")
			self.motor_lockout = 0
			self.set_estop(0)
			
		self.left_integrator = left_power
		self.right_integrator = right_power

		self.max_left_integrator = max(self.max_left_integrator, self.left_integrator)
		self.max_right_integrator = max(self.max_right_integrator, self.right_integrator)
		
		


		#print self.left_integrator, self.right_integrator

		return self.currents
	
	def _get_Errors(self):
		'''
		Gets the error status of the motor controllers
		
		return:
		error (int): Error code for motor controller
		
		'''
		self._lock_thread(1)
		self.error = self.rc.ReadError(self.address)[1]
		self._lock_thread(0)
		return self.error
	
	def _get_Encs(self):
		'''
		Gets the encoder values of the motor controllers
		
		return:
		list [2] (int): Speed of motors in radians/s, computed at each delta T
		
		'''
		self._lock_thread(1)
		r_enc = self.rc.ReadEncM1(self.address)[1]
		l_enc = self.rc.ReadEncM2(self.address)[1]
		dt = datetime.now()
		self._lock_thread(0)
		
		if self.prev_tick == [None, None]:
			l_vel_rad_s, r_vel_rad_s = 0,0
			self.timestamp = 0
		else:
			delta_t = ((dt-self.prev_enc_ts).microseconds)/1000000.0
			self.timestamp = int(self.timestamp + (delta_t  * 100000))
			l_vel = (l_enc - self.prev_tick[0])/(delta_t)
			r_vel = (r_enc - self.prev_tick[1])/(delta_t)
			
			l_vel_rad_s = l_vel * (2 * math.pi/self.tick_per_rev)
			r_vel_rad_s = r_vel * (2 * math.pi/self.tick_per_rev)
		
			l_vel_rad_s = int(l_vel_rad_s * 1000)/1000.0
			r_vel_rad_s = int(r_vel_rad_s * 1000)/1000.0
		
		self.prev_enc_ts = dt
		self.prev_tick = [l_enc, r_enc]
		self.enc = [l_vel_rad_s, r_vel_rad_s]
		
		return l_vel_rad_s, r_vel_rad_s

	def _send_motor_cmds(self):

		l_vel = self._l_vel_cmd_buf
		r_vel = self._r_vel_cmd_buf

		l_vel *= (self.tick_per_rev/(2*math.pi))
		r_vel *= (self.tick_per_rev/(2*math.pi))

		l_vel = max(min(l_vel,self.max_cts_per_s), -self.max_cts_per_s)
		r_vel = max(min(r_vel,self.max_cts_per_s), -self.max_cts_per_s)

		if (( abs(l_vel) <= self.max_cts_per_s) and (abs(r_vel) <= self.max_cts_per_s)):
			if not self.motor_lockout:
				self._lock_thread(1)
				self.rc.SpeedAccelM1(self.address, self.max_accel, int(r_vel))
				self.rc.SpeedAccelM2(self.address, self.max_accel, int(l_vel))
				self._lock_thread(0)
		else:
			rospy.loginfo( "values not in accepted range" )
			self.send_motor_cmds(0,0)

	# Public Methods

	def set_estop(self,val):
		'''
		Sets the E-stop pin to stop Motor control movement until cleared
		
		Parameters:
		val (int): 
			0 - Clears E-stop
			1 - Sets E-stop, disabling motor movement

		no return value
		
		'''

		if (val != self.e_stop):
			if val:
				rospy.loginfo( "Enabling the E-stop")
				GPIO.output(self.e_stop_pin, 1)
				self.e_stop = 1
			else:

				rospy.loginfo( "Clearing the E-stop")
				GPIO.output(self.e_stop_pin, 0)
				self.e_stop = 0


	def battery_state_esimator(self):
		x = self.voltage
		y = math.pow(x,2) * self.battery_coef_a + self.battery_coef_b * x + self.battery_coef_c
		self.battery_percent = (100 * y/float(self.battery_max_time))
		if (self.battery_percent <= self.battery_warning):
			self.shutdown_warning = True
		else:
			self.shutdown_warning = False
		if (self.battery_percent <= self.battery_shutdown):
			self.shutdown_flag = True
			self.set_estop(1)


	def update_cmd_buf(self, l_vel, r_vel):
		'''
		Updates the command buffers and sets the flag that new command has been received
		
		Parameters:
		l_vel (int): [-0.833, 0.833] units of [rad/s]
		r_vel (int): [-0.833, 0.833] units of [rad/s]
		
		no return value
		'''
		self._l_vel_cmd_buf = l_vel
		self._r_vel_cmd_buf = r_vel
		self._cmd_buf_flag = 1
			
	def kill_motors(self):
		'''
		Stops all motors on the assembly
		'''
		self._lock_thread(1)
		self.rc.ForwardM1(self.address, 0)
		self.rc.ForwardM2(self.address, 0)
		self._lock_thread(0)

	def loop(self, counter):
		'''
		Gets the data from the motor controllers to populate as class variables
		all data function classes are private because of threadlocks
	
		Downsample non-critical values to keep serial comm line free

		Parameters:
		counter (int): 

		no return value
		
		'''

		if (self._cmd_buf_flag):
			self._send_motor_cmds()
			self._cmd_buf_flag = 0

		self._get_Encs()
		if not counter % 2:
			self._get_Currents()
		if not counter % 5:
			self._get_Errors()
		if not counter % 20:
			self._get_Temp()
			self._get_Voltage()
			self.battery_state_esimator()

	def enable_12v_reg(self, en):
		'''
		Turns on/off the 12V regulator GPIO pin
		
		Parameters:
		en (int):
			0: Disabled
			1: Enabled
		
		'''
		try:
			if (en != self.reg_enabled):
				if en:
					rospy.loginfo("Enabling the 12V Regulator")
					GPIO.output(self.reg_en_pin, 1)
					self.reg_enabled = 1
				else:
					rospy.loginfo("Disabling the 12V Regulator")
					GPIO.output(self.reg_en_pin, 0)
					self.reg_enabled = 0
		except:
			pass

	def cleanup(self):
		'''
		Cleans up the motor controller node, stopping motors and killing all threads 

		no return value
		
		'''
		rospy.loginfo("Cleaning up the motor_controller node..")
		self._thread_lock = False
		self.kill_motors()
		self.set_estop(1)
		try:
			GPIO.cleanup()
		except:
			pass






