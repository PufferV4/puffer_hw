from roboclaw import Roboclaw
import serial

rc = Roboclaw("/dev/ttyTHS1", 115200)
rc.Open()
rc.ReadVersion(0x80)
