#include <Servo.h>

// Arduino sketch that drives a 2D delta robot to desired x, y coordinates
// Tamás Magyar 31-10-2017

//
// Constants:

const double  dist = 100; // distance between the two motors
const double  l1 = 150;  // length of the base arms (between a motor and a forearm)
const double  l2 = 300; // length of the forearms (that connect with the endeffector)

//
// Variables:

Servo servoR;
Servo servoL;
const int buttonPin = 2;     // the number of the pushbutton pin
int buttonState = 0;         // variable for reading the pushbutton status
volatile bool stop = false;

//
// Typedefs:
struct Point{
	double  x;
	double  y;
};

struct Circle{
	// center point and radius
	double  x;
	double  y;
	double  r;
};

struct ServoAngles{
	double  theta1;
	double  theta2;
};



//
// Function headers:

Point * intersection(Circle c1, Circle c2, bool & outOfRange); // calculates the intersection points between two circles.
															   // if there is no intersection, outOfRange is set to true.
ServoAngles inverse(Point p, bool &outOfRange);	// calculates base arm angles theta1 and theta2 from on the endpoint position.
												// if it is unreachable by the robot, outOfRange is set to true.

void buttonPressed();
bool turnServo(int servoNum, int duration); //depracated
float x, y; // for user input

// for inverse kinematics:
Point p;	
bool outOfRange = false;
ServoAngles angles;

void setup(){
	
	servoR.attach(9);
	servoL.attach(10);
	
	// go to start position (base arms horizontal):
	servoR.write(90);
	servoL.write(90);
	delay(1000);
	
	// set Led and Button IO:
	pinMode(LED_BUILTIN, OUTPUT);
	pinMode(buttonPin, INPUT);
	attachInterrupt(0,buttonPressed, HIGH); // (Interrupt number, Interrupt_Service_routine, Rising-Falling?)
	
	
	// Serial communication: 
	Serial.begin(115200);
	while(!Serial){}
	Serial.println("Enter position x (min:0, max: +-150): ");
	while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
	x = Serial.parseFloat();  // SLOW!! Needs to be rewritten!
	
	Serial.println("Enter position y: (min 160, max 400): ");
	while (Serial.available() && Serial.read()); // empty buffer again
	while (!Serial.available());                 // wait for data
	y = Serial.parseFloat();  // SLOW!! Needs to be rewritten!
	Serial.println(x);
	Serial.println(y);
	// x, y is entered by the user
	
	
	
	
	
	//ˇˇˇˇThe rest of the setup moves the robot on a rectanglular path.ˇˇˇˇˇ
	
	// Inverse calculation:
	outOfRange = false;
	p.x = 70;
	p.y = 250;
	angles = inverse(p, outOfRange);
	Serial.println(angles.theta1);
	Serial.println(angles.theta2);
	
	// Drive servos:
	servoL.write(int(180-angles.theta1));
	servoR.write(int(angles.theta2));
	delay(1000); // wait for the servos to arrive at the position.
	// ^^^^^TODO: arrange this into a movej(x, y) function^^^^^^
	
	// Inverse calculation:
	outOfRange = false;
	p.x = 70;
	p.y = 350;
	angles = inverse(p, outOfRange);
	Serial.println(angles.theta1);
	Serial.println(angles.theta2);
	
	// Drive servos:
	servoL.write(int(180-angles.theta1));
	servoR.write(int(angles.theta2));
	delay(1000); // wait for servos to arrive at the position.
	
	// Inverse calculation:
	outOfRange = false;
	p.x = -70;
	p.y = 350;
	angles = inverse(p, outOfRange);
	Serial.println(angles.theta1);
	Serial.println(angles.theta2);
	
	// Drive servos:
	servoL.write(int(180-angles.theta1));
	servoR.write(int(angles.theta2));
	delay(1000); // wait for servos to arrive at the position.
	
	// Inverse calculation:
	outOfRange = false;
	p.x = -70;
	p.y = 250;
	angles = inverse(p, outOfRange);
	Serial.println(angles.theta1);
	Serial.println(angles.theta2);
	
	// Drive servos:
	servoL.write(int(180-angles.theta1));
	servoR.write(int(angles.theta2));
	delay(1000); // wait for servos to arrive at the position.
	
	
	// Inverse calculation:
	outOfRange = false;
	p.x = 70;
	p.y = 250;
	angles = inverse(p, outOfRange);
	Serial.println(angles.theta1);
	Serial.println(angles.theta2);
	
	// Drive servos:
	servoL.write(int(180-angles.theta1));
	servoR.write(int(angles.theta2));
	delay(3000); // wait for servos to arrive at the position.
	
	servoL.write(90);
	servoR.write(90);
	
}

void loop(){
	
	delay(500);
	
}

//Function definitions:

//Interrupt routine that stops the program on button press (not implemented yet in the new version)
void buttonPressed(){
	//digitalWrite(LED_BUILTIN, LOW);
	
	stop = true;
}

// TODO: get rid of this depracated function
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

Point * intersection(Circle c1, Circle c2, bool & outOfRange){
	outOfRange = false;
	Point * outputPoints = (Point*)malloc(2*sizeof(Point)); // allocate memory for the outpoints[] array that we will return
	double  x1 = c1.x;
	double  y1 = c1.y;
	double  r1 = c1.r;
	double  x2 = c2.x;
	double  y2 = c2.y;
	double  r2 = c2.r;
	// P1, P2: center points, I1, I2: intersection points
	// L: line connecting the center points
	double  d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
	if (!(abs(r1 - r2) <= d && d <= r1 + r2)) { // no intersection
		outputPoints[0].x = -1;	
		outputPoints[0].y = -1;
		outputPoints[1].x = -1;
		outputPoints[1].y = -1;
		return outputPoints;
		
	 // empty list of results
	}
		 // intersection(s) should exist
	double  a = (r1*r1-r2*r2+d*d)/(2*d); // length of the orthogonal projection of P1_I1 vector to L (as well as P1I2 to L) 
	double  h = sqrt(r1*r1-a*a); // length of the orthogonal complement of P1_I1 vector to L
	double  xc = x1+(a/d)*(x2-x1); // Endpoint x of the orthogonal projection of P1_I1 vector to L
	double  yc = y1+(a/d)*(y2-y1); // Endpoint y of the orthogonal projection of P1_I1 vector to L
	// I1, I2:
	double  xi1 = xc + (h/d)*(y2-y1);	// x coordinate of the first intersection point
	double  yi1 = yc - (h/d)*(x2-x1);	// y coordinate of the first intersection point
		
	double  xi2 = xc - (h/d)*(y2-y1);	// y coordinate of the second intersection point
	double  yi2 = yc + (h/d)*(x2-x1);	// y coordinate of the second intersection point
	
	//TODO: get rid of this depracated if-else statement
	if (yi1 > yi2){
		// fills the outpoints[] array with the calculated intersections 
		// (we have two intersection points, outputpoints[0] and outputPoints[1])
		outputPoints[0].x = xi1;	
		outputPoints[0].y = yi1;
		outputPoints[1].x = xi2;
		outputPoints[1].y = yi2;
	}
	else{
		outputPoints[1].x = xi1;	
		outputPoints[1].y = yi1;
		outputPoints[0].x = xi2;
		outputPoints[0].y = yi2;
	}
	
	return outputPoints;
}





ServoAngles inverse(Point p, bool &outOfRange){
	
	ServoAngles servoAngles; // This will store the results of the calculation (theta1, theta2)
	servoAngles.theta1 = 0;
	servoAngles.theta2 = 0;
	
	Circle cBaseL, cBaseR, cEnd; 
	
	// Right base circle: center is the right motor axis, radius is l1
	cBaseR.x = dist/2;
	cBaseR.y = 0;
	cBaseR.r = l1;
	
	// Left base circle: center is the left motor axis, radius is l1
	cBaseL.x = (-1)*dist/2;
	cBaseL.y = 0;
	cBaseL.r = l1;
	
	// End effector circle: center is the end effector position (p.x, p.y), radius is l2
	cEnd.x = p.x;
	cEnd.y = p.y;
	cEnd.r = l2;
	
	// find intersections between the left base circle and the end effector circle:
	Point * intersect_pointsL;
	intersect_pointsL = intersection(cBaseL, cEnd, outOfRange); // this finds the two possible positions of the left "elbow"
	// select the desired position out of the two possibilities:
	Point LeftArm;
	if(intersect_pointsL[0].x < intersect_pointsL[1].x){ 
		LeftArm = intersect_pointsL[0];
	}
	else{
		LeftArm = intersect_pointsL[1];	
	}
	
	
	// find intersections between the right base circle and the end effector circle:
	Point * intersect_pointsR;
	intersect_pointsR = intersection(cBaseR, cEnd, outOfRange); // this finds the two possible positions of the right "elbow"
	
	
	// select the desired position out of the two possibilities:
	Point RightArm;
	if(intersect_pointsR[0].x > intersect_pointsR[1].x){
		RightArm = intersect_pointsR[0];
	}
	else{
		RightArm = intersect_pointsR[1];
	}
	
	// The intersection function sets outOfRange to true if the two circles do not touch
	if(outOfRange == true){ // we selected a point that is out of the work area
		// Set the angles to a safe value
		servoAngles.theta1 = 90; 
		servoAngles.theta2 = 90;
	}
	else{ // The position is in the work area, so we calcualte the theta angles with atan2(x_elbow,y_elbow)
	
	// we ahve two sides of a triangle, we find the angle between the two sides:
	servoAngles.theta1 = -(180/M_PI)*atan2(LeftArm.x+dist/2, LeftArm.y); 
	// we ahve two sides of a triangle, we find the angle between the two sides:
	servoAngles.theta2 = (180/M_PI)*atan2(RightArm.x-dist/2 , RightArm.y); 
	
	}	
	// Free dinamically allocated memory
	free(intersect_pointsL);
	free(intersect_pointsR);
	
	return servoAngles;
}
