#include <Ultrasonic.h>


//TimeOut calculation
// TimeOut = Max.Distance(cm) * 58 ;  For 215cm * 58 = 13050
// From: https://github.com/JRodrigoTech/Ultrasonic-HC-SR04/wiki/How-to-change-the-default-tiemout%3F

//ArduinoMega Top row (Even No)= Trig pins
//ArduinoMega Bottom row (Odd No)= Echo pins
// (Trig PIN,Echo PIN, Timeout)

//  sensor (counterclockwise) order: L2, L1, C, R1 and R2
// %   (x = SR-04)
// %              C
// %            --x--
// %     L1     x    x     R1
// %            |    |
// %     L2     x    x     R2
// %            ------

Ultrasonic SR04_Left2(36,37, 13050);
Ultrasonic SR04_Left1(34,35, 13050);
Ultrasonic SR04_Center(32,33, 13050);
Ultrasonic SR04_Right1(30,31, 13050);
Ultrasonic SR04_Right2(28,29, 13050);

int dist1, dist2, dist3, dist4, dist5;
char checkCond;

void setup()
{
	Serial.begin(9600);
}

void loop()
{
	if (Serial.available() > 0)
	{
		// read the command
		checkCond = Serial.read();
		if(checkCond == 'R')
		{
		    getvalues();	// Get values from all sensors
			sendvalues();	// Transmit over serial/USB
		}
		// send command to motors
		else if(checkCond == 'W')
		{
		    // statement
		}
	}
}

void getvalues()
{
	dist1 = SR04_Left2.Ranging(CM); // CM or INC
	delay(30);	// delay in ms
	dist2 = SR04_Left1.Ranging(CM);
	delay(30);
	dist3 = SR04_Center.Ranging(CM);
	delay(30);
	dist4 = SR04_Right1.Ranging(CM);
	delay(30);
	dist5 = SR04_Right2.Ranging(CM);
	delay(30);
}

void sendvalues()
{
	// Print sensors values on Serial port (USB COM port)
	// seperate all values by Spaces
	Serial.print(dist1);
	Serial.print(" ");
	Serial.print(dist2);
	Serial.print(" ");
	Serial.print(dist3);
	Serial.print(" ");
	Serial.print(dist4);
	Serial.print(" ");
	Serial.print(dist5);
	Serial.println("");	// end with a line break (?)
}
