Code Documentation for Exoskeleton Arm
Begun 5-19-2021

_____________________________________________________________________________________


// w/b Trevor K. Carter 5-19-2021
//Using Arduino Uno to read two potentiometers and control linear actuator


// Pins to read the potentiometers and a button
int manPotPin = 5; //Anlg. pin, read manual arm potentiometer
int powPotPin = 4; //Anlg. pin, read powered arm potentiometer
int butPin = 2; // Dgtl. pin, read button

// Pins to control the motor’s speed and direction
int motDir1Pin = 0; // Dgtl. pin, HIGH w/ motDir2 LOW for forward, connect to IN1 and IN3
int motDir1Pin = 1; // Dgtl. pin, HIGH w/ motDir1 LOW for backward, connect to IN2 and IN4
int motSpeedPin = 3; // Dgtl. PWM pin, 0-255 to control speed, connect to ENA and ENB

// Values for program settings
// Minimum and maximum mechanically possible values to be read from the potentiometers
int potReadMin = 0;
int potReadMax = 1023;
// The forward motor direction is up, as one would perform a bicep curl
// The potentiometer is close to its maximum value when the manual is above the powered arm
// The potentiometer is close to its minimum value when the manual is below the powered arm
// Buffer to allow when determining if the arms are separate enough to power the motors
int buffer = 16;
// Potentiometer range over which to adjust the speed (difference more than this = max speed)
int speedRange = 64;

// Variables used in the program
int motSpeed = 0;


void setup() {
	// These two will only output HIGH or LOW
	pinMode(motDir1Pin, OUTPUT);
pinMode(motDir2Pin, OUTPUT);

// For reading if the button is on or off
//pinMode(butPin, INPUT);

// This pin will output a PWM signal from 0-255 with the command analogWrite(pin, num)
pinMode(motSpeedPin, OUTPUT);
}


void loop() {
	// If the button is pressed, stay still!
	if(digitalRead(butPin) == HIGH) {
		analogWrite(motSpeedPin, 0);
		return;
}

	// Determine the distance between the exoskeleton arms.
// If using two potentiometers, find their difference, then convert it to be read as a single potentiometer
	int potVal = 0;
	potVal = (1023+analogRead(manPotPin)-analogRead(powPotPin))/2
	
	// Map the measured potentiometer values (based on possible min/max) to 0 to 1023
	map(potVal, potReadMin, potReadMax, 0, 1023);
	
	// If the manual arm is below the powered arm (0-511), drive the motor backward
	if (potVal <= 511-buffer) {
		digitalWrite(motDir1Pin, LOW);
		digitalWrite(motDir2Pin, HIGH);

		// If the potentiometer is within the speed range, adjust the speed
		if (potVal >= 511-speedRange) {
			motSpeed = 255*(511-potVal)/speedRange;
			analogWrite(motSpeedPin, motSpeed);
		}
// Otherwise, drive it at maximum speed.
		else {
			analogWrite(motSpeedPin, 255);
		}
	}
	// If the manual arm is above the powered arm (512-1023), drive the motor forward
	else if (potVal >= 512+buffer) {
		digitalWrite(motDir1Pin, HIGH);
		digitalWrite(motDir2Pin, LOW);

		// If the potentiometer is within the speed range, adjust the speed
		if (potVal <= 512+speedRange) {
			motSpeed = 255*(potVal-512)/speedRange;
			analogWrite(motSpeedPin, motSpeed);
		}
// Otherwise, drive it at maximum speed.
		else {
			analogWrite(motSpeedPin, 255);
		}
	}
	// If the the arm’s difference isn’t greater than the buffer, stay still.
	else {
		analogWrite(motSpeedPin, 0);
	}


		
}



---------------------------------------------------------------------------------------------




// FINAL VERSION ON ROBOT
// w/b Trevor K. Carter 6-2-2021
//Using Arduino Uno to read two potentiometers and control linear actuator




// Pins to read the potentiometers and a button
int manPotPin = 5; //Anlg. pin, read manual arm potentiometer
int powPotPin = 4; //Anlg. pin, read powered arm potentiometer
//int butPin = 2; // Dgtl. pin, read button


// Pins to control the motor’s speed and direction
int motDir1Pin = 0; // Dgtl. pin, HIGH w/ motDir2 LOW for forward, connected to both EN2 and EN3
int motDir2Pin = 1; // Dgtl. pin, HIGH w/ motDir1 LOW for backward, connected to both EN1 and EN4
int motSpeedPin = 3; // Dgtl. PWM pin, 0-255 to control speed, connected to both to ENA and ENB
/*
 Motor Pin: | motDir1: | motDir2:
  0V        |  HIGH    |  HIGH
  0V        |  LOW     |  LOW
 +12V       |  HIGH    |  LOW
 -12V       |  LOW     |  HIGH


The "Motor Pin" is the output power pins on the side of the board with the power-in terminals, where the red motor wires attach.
When the "Motor Pin" recieve +12V, the actuator will retract. When it recieves -12V, it will extend.
The "motDir1" pin is connected to both EN2 and EN3, "motDir2" is connected to both EN1 and EN4.
*/




// Minimum and maximum mechanically possible values to be read from the potentiometers
int manPotMin = 325;    // Experimentally found to be ~322
int manPotMax = 655; // Experimentally found to be ~650
int powPotMin = 345;    // Experimentally found to be ~345
int powPotMax = 670; // Experimentally found to be ~670
// Each potentiometer is closer to its maximum value when it is more curled




// Buffer to allow when determining if the arms are separate enough to power the motors
int buffer = 20;




// Motor speed initial condition when the program starts
int motSpeed = 0;






void setup() {


  // These two will only output HIGH or LOW
  pinMode(motDir1Pin, OUTPUT);
  pinMode(motDir2Pin, OUTPUT);


  // This pin will output a PWM signal from 0-255 with the command analogWrite(pin, num)
  pinMode(motSpeedPin, OUTPUT);
}




void loop() {


  // Convert each potentiometer to a scale from 0-1023
  int potVal = 0;
  int manPotConv = map(analogRead(manPotPin), manPotMin, manPotMax, 0, 1023);
  int powPotConv = map(analogRead(powPotPin), powPotMin, powPotMax, 0, 1023);


  // Find the difference between the scaled potentiometer positions (negative when manual is more extended than powered)
  potVal = manPotConv-powPotConv;




  // If the manual arm is less curled than the powered arm...
  if (potVal <= 0-buffer) {
    // Retract the linear actuator to make the powered arm less curled
    digitalWrite(motDir1Pin, HIGH);
    digitalWrite(motDir2Pin, LOW);
    analogWrite(motSpeedPin, 255);
  }
  // If the manual arm is more curled than the powered arm...
  else if (potVal > 0+buffer) {
    // Extend the linear actuator to make the powered arm more curled
    digitalWrite(motDir1Pin, LOW);
    digitalWrite(motDir2Pin, HIGH);
    analogWrite(motSpeedPin, 255);
  }
  // If it's not significantly different...
  else {
    // Extend the linear actuator to make the powered arm more curled
    analogWrite(motSpeedPin, 0);
  }


}

