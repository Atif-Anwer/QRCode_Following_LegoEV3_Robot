# QR Code Following Robot Using Lego Mindstorms EV3, Matlab, Arduino and a Webcam #

### What is this code for? ###
The repository is the Matlab code for controlling a robot made with Lego Mindstorms EV3 and controlled with Matlab over a USB cable/Bluetooth dongle. The design of the lego robot is given as below:

### Video Results  ###

[Video Tracking Results](https://youtu.be/pOSssa08bpE) Can be seen at my youtube channel. 
[![Video Tracking Results](https://img.youtube.com/vi/pOSssa08bpE/0.jpg)](https://www.youtube.com/watch?v=pOSssa08bpE) Can be seen at my youtube channel

### Hardware and Software requirements? ###

The following hardware are required for use
* 1x LEGO EV3
* 2x Lego Large Motors (EV3)
* 1x Lego Ultrasonic Transeiver (EV3)
* 1x Webcam (Logitech C270 used)
* 1x Arduino MEGAADK
* 5x HC-SR04 Ultrasonic Sensors
* Tested on Matlab 2016a and later

### Lego Robot Assebly and design ###

![Lego Robot Design](https://i.imgur.com/FAHR3qDl.jpg). 

The design was based on the Ultrasonic sensor driving base design from lego. The PDF design instructions can be downloaded from [Lego Website](https://le-www-live-s.legocdn.com/sc/media/lessons/mindstorms-ev3/building-instructions/ev3-ultrasonic-sensor-driving-base-61ffdfa461aee2470b8ddbeab16e2070.pdf)

### Basic Code Flow ###

The code flow can be generalized as follows:
	
1. Get Deviation from center and Direction from KLT tracking
	a. Find Region of Interest and its center (4x Red Circles in a square) 
	b. Find SURF Feature points (QR Code with 4x Red Circles at the corners)
	c. Match SURF features with QR Code template in file
	d. Continue if QR code found else stop moving
2. Get Values of 5x Ultrasonic sensors from Arduino
3. Get value from 1x front LEGO ultrasonic sensor
4. Calculate Direction to turn based on the position of QR code in the field of view
5. Calculate speed of turn for both motors using PID feedback control
6. Save values to pass on to cartTurn function to execute the values
7. Check if the main window is open. If its closed; exit the program.
