#include <Servo.h>



// Variables:

Servo servoR;
Servo servoL;
const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
volatile bool stop = false;

// Function headers:

void buttonPressed();
bool turnServo(int servoNum, int duration);


void setup(){
	servoR.attach(9);
	servoL.attach(10);
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(buttonPin, INPUT);
	attachInterrupt(0,buttonPressed, HIGH); // (Interrupt number, Interrupt_Service_routine, Rising-Falling?)
}

void loop(){
	
	digitalWrite(LED_BUILTIN, HIGH);
	
	
	if (stop == false) turnServo(servoR,5);
	if (stop == false) turnServo(servoL,5);
	delay(500);
	if (stop == false) turnServo(servoR,-5);
	if (stop == false) turnServo(servoL,-5);
	delay(500);
	servoR.write(90);
	digitalWrite(LED_BUILTIN, LOW);
	delay(500);

}

//Function definitions:

void buttonPressed(){
	//digitalWrite(LED_BUILTIN, LOW);
	
	stop = true;
}

bool turnServo(Servo &servo, int duration){
	int speed = 20; // a value between 0 and 90
	bool done = false;
	int ticks = 0;
	int direction =1;
	if (duration < 0){
		direction = -1;
		
	}
	ticks = duration*direction;
	while( ticks >= 0 && stop==false ){
		servo.write(90+speed*direction);
		delay(15);
		ticks--;
		
	}
	servo.write(90);
	done = true;
	return done;
}


