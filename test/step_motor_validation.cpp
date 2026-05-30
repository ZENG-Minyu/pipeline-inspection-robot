#include <Arduino.h>
// Define pin connections & motor's steps per revolution
const int dirPinX = 5;
const int stepPinX = 2;
const int dirPinY = 6;
const int stepPinY = 3;
const int dirPinZ = 7;
const int stepPinZ = 4;
const int enablePin = 8;
const int stepsPerRevolution = 200;

void setup()
{
	// Declare pins as Outputs
	pinMode(stepPinX, OUTPUT);
	pinMode(dirPinX, OUTPUT);
	pinMode(stepPinY, OUTPUT);
	pinMode(dirPinY, OUTPUT);
	pinMode(stepPinZ, OUTPUT);
	pinMode(dirPinZ, OUTPUT);
	pinMode(enablePin, OUTPUT);
	digitalWrite(enablePin, LOW);

	Serial.begin(115200);
}
void loop()
{
	// Set motor direction clockwise
	digitalWrite(dirPinX, HIGH);
	digitalWrite(dirPinY, HIGH);
	digitalWrite(dirPinZ, HIGH);
	Serial.println("Running clockwise...");

	// Spin motor slowly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPinX, HIGH);
		digitalWrite(stepPinY, HIGH);
		digitalWrite(stepPinZ, HIGH);
		delayMicroseconds(500);
		digitalWrite(stepPinX, LOW);
		digitalWrite(stepPinY, LOW);
		digitalWrite(stepPinZ, LOW);
		delayMicroseconds(500);
	}
	delay(1000); // Wait a second
	
	// Set motor direction counterclockwise
	digitalWrite(dirPinX, LOW);
	digitalWrite(dirPinY, LOW);
	digitalWrite(dirPinZ, LOW);
	Serial.println("Running counterclockwise...");

	// Spin motor quickly
	for(int x = 0; x < stepsPerRevolution; x++)
	{
		digitalWrite(stepPinX, HIGH);
		digitalWrite(stepPinY, HIGH);
		digitalWrite(stepPinZ, HIGH);
		delayMicroseconds(500);
		digitalWrite(stepPinX, LOW);
		digitalWrite(stepPinY, LOW);
		digitalWrite(stepPinZ, LOW);
		delayMicroseconds(500);
	}
	delay(1000); // Wait a second
}
