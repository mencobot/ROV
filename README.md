# Arduino ROV
ROV powered by 8 x bldc motor Thrusters (4 vertical & 4 horizontal thruster arrangement).
Both Top & Bottom side pcbs are controlled by Arduino Mega 2560 MCUs
ROV locomotion and peripherals controlled by PS2 controller connected to the Top side
All environmental data sent by the ROV is displayed on 32x128 oled display at Top side.
ROV sends onboard and overboard temperature, pressure, depth, roll, pitch and yaw data
along with live video feed from ROV's onboard ip camera to the Top side.
Camera & Lights on/off and camera tilt & pan servos and light intensity to be
controlled through ps2 game controller and potentiometer attached to TOP side control unit.
Full duplex communivcation between Top & ROV sides are through a pair
of RS422/Max490 boards. Cat6 wire Tether connects Top & ROV sides.
Top side shields:
SmartElecx ps2 shield
oled 32x128
ROV side sensors:
onboard barro-bmp280
imu-mpu6050
overboard barro-ms5540c
3-axis compass-hmc5883l
For more information email us at menco.mec@gmail.com
