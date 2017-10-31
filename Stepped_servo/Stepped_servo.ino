#include <Servo.h>

// Arduino sketch that allows us to control a servo similarly to a stepper motor
// Tamás Magyar 31-10-2017

//
// Constants:



//
// Typedefs:

class SteppedServo // Instead of Servo::write(angle) we can now use SteppedServo::step(number of steps)
{
	
	public:
		SteppedServo();
		int read();
		void write(int pos);
		void init(Servo & sr);
		void step(int direction); 
	private:
		Servo its_servo;
};

SteppedServo::SteppedServo(){
}


// This is necessary because of the setup -- loop construct. Other solution would be to use a while loop inside the setup.
void SteppedServo::init(Servo & sr){   // This is basically the constructor, that way it can be called in setup(){}
	its_servo = sr;					  // sr needs to be already attached to a pin when calling this function. attach() can only be called in setup(){}. 
}						// (If we declare the servo in the setup and use the constructor to initialize, the servo is not visible in the loop!)

int SteppedServo::read(){
	return its_servo.read(); // Reads the last position written to the servo.
}

void SteppedServo::write(int pos){
	its_servo.write(pos);
}

void SteppedServo::step(int steps){  
	int currentPos = its_servo.read();
	int newPos = currentPos+steps;
	if (newPos>174){ // limit 
		newPos = 174;
	}
	if (newPos<6){ // limit
		newPos = 6;
	}
	its_servo.write(newPos);
}

//
// Function headers:

//
// Variables:

// Global servo variables
Servo sR;
Servo sL;

// Create global servo objects

SteppedServo servoR; // SteppedServo servoR() <-- compiler thinks it is a function definition, so no parenthesis!
SteppedServo servoL; 

void setup(){
	
	// First we need to attach the servo objects sR, sL to pins 9, 10 in the setup
	sR.attach(9); 
	sL.attach(10);
	
	// Then we can init our steppedServos servoR and servoL with sR and SL 
	servoR.init(sR);
	servoL.init(sL);
	
	// They the original servo class methods are available
	servoL.write(90);
	servoR.write(90);
	delay(1000);
}

int startPos;
int goalPos;
int currentPos;

void loop(){
	delay(1000);
	servoR.write(10);
	delay(1000);
	startPos = servoR.read();
	goalPos = 170;

	while(servoR.read()!=170){
		currentPos = servoR.read();
		if (goalPos-servoR.read()!=0){
			float difference = goalPos-currentPos;
			int delayTime = 0.05*difference;
			servoR.step(1);
			delay(delayTime);
		}
		else break;
	}
	
}