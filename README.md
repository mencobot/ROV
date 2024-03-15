# Arduino ROV
Our ROV to be powered by 6 x bldc motor Thruster configurations.
Both DRY(Top) & WET(ROV) side pcbs to be controlled by Arduino Mega boards
ROV to be controlled by PS2 controller connected to the Top side
All data sent by the ROV to be displayed on 16x2 lcd displat at DRY side
ROV to send onboard and overboard temperature, pressure, depth, and heading data
along with live video feed from ROV onboard ip camera to DRY(Top) side.
Camera & Lights on/off and camera tilt servo and light intensity to be
controlled through ps2 game controller and potentiometer 
attached to DRY(TOP) side control unit.
Full duplex communivcation between DRY & WET sides are through a pair
of RS422/Max490 boards.
Cat5e or Cat6 wire to act as Tether connecting  DRY & WET sides.
