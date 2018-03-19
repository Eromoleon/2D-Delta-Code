/*
Delta robot arduino control code
2018-03-19
*/


#include <cppQueue.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ##############################
// Global constants::

// ..geometry:

#define START_ANGLE_LEFT 0
#define START_ANGLE_RIGHT 0
#define PICK_HEIGHT 200
const double  dist = 100; // distance between the two motors
const double  l1 = 150;  // length of the base arms (between a motor and a forearm)
const double  l2 = 300; // length of the forearms (that connect with the endeffector)

// ..other:

#define MAGNET_PIN 1
#define COMMAND_LENGTH 10
#define FIFO_SIZE 20
#define STEPS_REVOLUTION 200

// ##############################
// Typedefs:

struct Command  {
  int determinant;
  int data[COMMAND_LENGTH];
};

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

struct MotorAngles{
  double  theta1;
  double  theta2;
};

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_StepperMotor *motL = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *motR = AFMS.getStepper(200, 2);

class Actuators{
	public:
		Actuators(Adafruit_StepperMotor *mL, Adafruit_StepperMotor *mR, int magnetP);
		bool taskMgr(Command c);
		bool move(float x, float y);
		bool pickAndP(int xCoord = 0, int color = 0);
		bool moveJ(float stepsL, float stepsR);
		bool calibration();
		bool setMagnet(bool status);
		bool inverseK(int xCoord, int yCoord);
		bool directK(int thetaL, int thetaR);
		int moveLeft(int steps);
		Point * intersection(Circle c1, Circle c2, bool & outOfRange);
		MotorAngles inverse(Point p, bool &outOfRange);
		
	private:
	// TODO 4 points that constrain the work area
		Adafruit_StepperMotor*  motorL;
		Adafruit_StepperMotor*  motorR;
		int magnetPin;		
		bool outOfReach;
		float angleR = 0;
		float angleL = 0;
		
};


// Functions: 
bool taskMgr(Command c);


// Global variables:

Queue commandQ(sizeof(Command), FIFO_SIZE, FIFO);
int magnetP = MAGNET_PIN;
MotorAngles angles;
Actuators actuators(motL, motR, magnetP);
bool outOfRange = false;
String inputString = "";         // a String to hold incoming data



void setup() {
	 // Arduino digital pin  used to control the electromagnet
	// Stepper shield setup:
	AFMS.begin();  // create with the default frequency 1.6KHz
	//AFMS.begin(1000);  // OR with a different frequency, say 1KHz
	
	
	// initialize serial:
	Serial.begin(115200);
	// reserve 200 bytes for the inputString:
	inputString.reserve(200);
	
	
	// Calibration:
	actuators.calibration();
	
	// Debugging: 
	motL->setSpeed(150);
	motR->setSpeed(150);
	motL->step(200, FORWARD, SINGLE) ;
	motL->step(200, BACKWARD, SINGLE) ;
	
	motR->step(200, FORWARD, SINGLE) ;
	motR->step(200, BACKWARD, SINGLE) ;
	
	delay(2000);
	
	actuators.moveJ(70, 100);
	actuators.moveJ(-70, -100);
	actuators.moveJ(100, 70);
	actuators.moveJ(-100, -70);

	actuators.moveJ(100, 70);
	actuators.moveJ(-100, -70);
	actuators.moveJ(70, 100);
	actuators.moveJ(-70, -100);
	
	actuators.moveJ(0, -100);
	actuators.moveJ(-100, 0);
	
	actuators.moveJ(200,0);
	actuators.moveJ(0,200);
	
	actuators.moveJ(-50,-150);
	actuators.moveJ(-150,-50);
	Point p;
	p.x = 100; p.y = 300;
	MotorAngles angls = actuators.inverse(p, outOfRange);
	Serial.println("angles: ");
	Serial.println(angls.theta1);
	Serial.print( " ");
	Serial.println( angls.theta2);

} // __End setup__

void loop() {

  if(commandQ.isEmpty())
  {
    Serial.println("Command queue empty, waiting for command...");
    delay(1000);
  }
  else{

    Command c;
    commandQ.pop(&c);
    Serial.println("Command type: ");
    Serial.println(c.determinant);
	actuators.taskMgr(c);
  }

    delay(10);
}
// __End loop__


bool Actuators::pickAndP(int xCoord, int color){
	int yCoord = PICK_HEIGHT;
	
	Serial.println("Normal operation, piece sorting");
	Serial.print("Right Left Forward?: ");
	Serial.println(color);
	Serial.print("X: ");
	Serial.println(xCoord);
	Serial.println("Executing command...");
	inverseK(xCoord, yCoord-10);
	delay(2000);
	Serial.println("\n");
}

bool Actuators::move(float thetaL, float thetaR){
	float diffL = thetaL-angleL;
	float diffR = thetaR-angleR;
	
	float stepsL = diffL / (360 / STEPS_REVOLUTION);
	float stepsR = diffR / (360 / STEPS_REVOLUTION);
	
	moveJ(stepsL, stepsR);
	
	return true;
}

bool Actuators::inverseK(int xCoord, int yCoord){
	MotorAngles angles;
	Point p;
	p.x = xCoord;
	p.y = yCoord;
	angles = inverse(p, outOfRange);
	
	if(outOfRange == false){
		move(angles.theta1, angles.theta2);
	}
	
	return true;
}


bool Actuators::taskMgr(Command c){
	 switch(c.determinant){
		case 0:
			pickAndP(c.data[1], c.data[2]);
			break;
		case 1:
			inverseK(c.data[1], c.data[2]);
			break;
		case 2:
			directK(c.data[1], c.data[2]);
			break;
		case 3:
			calibration();
			break;
		default:
			Serial.println("Undefined command type.");
			break;
    }
    Serial.println("#########################################################");
}



void serialEvent() {
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/

  while (Serial.available()) {
    // get the new byte:
  String inString = "";

    char inChar = (char)Serial.read();
    inputString += inChar;
    
    if ( inChar == '\n' ) {
      int nCount = 0;
      Command c;
      for( int j = 0; j<COMMAND_LENGTH; j++ ){
        c.data[j] = 0; // Initialize the command array
      }


      String numString = "";
      for( size_t i = 0; i<inputString.length(); i++ ) {
			char ch = (char)inputString[i];
			long int number = 0;
			
			if(isDigit(ch)) {
				numString+=ch ;
			}
			else {
				number = numString.toInt();
				c.data[nCount] = (int)number;
				nCount++;
				numString = "";
			}
      }

      c.determinant = c.data[0] ;
      commandQ.push(&c);
      Serial.println("Message inserted into the Queue  \n");
      inputString = "";
    } // End if( ?"\n")
  }
} // End SerialEven



Actuators::Actuators(Adafruit_StepperMotor *mL, Adafruit_StepperMotor *mR, int magnetP){
	motorL = mL;
	motorR = mR;
	magnetPin = magnetP;
}



// TODO: correct inaccuracy by using half- and microstepping at the end of the movement
bool Actuators::moveJ(float stepsL, float stepsR){ // Quasi-simultaneous operation of the two motors (Both motors stop at the same time) with trapez profile
	
	// 1 or 0, for the step function: 
	int directionL = 1; 
	int directionR = 1;
	// 1 or -1 for counting the degrees turned:
	int dirL = 1;
	int dirR = 1;
	if(stepsL < 0){
		directionL = 0; // BACKWARD
		dirL = -1;
	}
	if(stepsR < 0){
		directionR = 0; // BACKWARD
		dirR = -1;
	} 
	stepsL = abs(stepsL);
	stepsR = abs(stepsR);
	
	int leftStepsTaken = 0;
	int rightStepsTaken = 0;
	float stepRatioL = 0.0;
	float stepRatioR = 0.0;
	float steps2TakeL = 0.0;
	float steps2TakeR = 0.0;
	float maxSteps = 0.0;
	// Left motor steps more, right motor steps one every time
	if(stepsL > stepsR){
		stepRatioR = stepsR / stepsL;
		stepRatioL = 1;
		maxSteps = stepsL;
	}
	else {
		stepRatioR  = 1;
		stepRatioL = stepsL / stepsR;
		maxSteps = stepsR;
	}
	Serial.println("Stepping left motor with ");
	Serial.print(stepsL); Serial.println(" steps.");
	Serial.println("Stepping right motor with ");
	Serial.print(stepsR); Serial.println(" steps.");
	
	// Trapeze profile parameters: 
	int minSpeed = 100;
	int maxSpeed = 600; // RPM
	int startSpeed = 1;
	//int t1R = int( round( stepsR / 3 ));
	//int t1L = int( round( stepsL / 3 ));
	int t1 = int( round( maxSteps /3 ) );
	//int speedR = 0;
	//int speedL = 0;
	int speed = minSpeed;
	//int speedIncrL = 1 + int(round((maxSpeed-startSpeed)/t1L));
	//int speedIncrR = 1 + int(round((maxSpeed-startSpeed)/t1R));
	int speedIncr = 1 + int(round((maxSpeed-startSpeed)/t1));
	motorL->setSpeed(1000);
	motorR->setSpeed(1000);
	for(int i = 0; i < int(round(maxSteps)); i++){
		if(i<t1&& speed < maxSpeed){
			speed+=speedIncr;
		}
		else if(i>int(round(maxSteps-t1)) && speed > minSpeed){
			speed-=speedIncr;
		}
		//motorL->setSpeed(speed);
		//
		//motorR->setSpeed(speed);
		steps2TakeL += stepRatioL;
		steps2TakeR += stepRatioR;
		int intStpL = int(round(steps2TakeL));
		int intStpR = int(round(steps2TakeR));
		if (intStpL == 1){
			motorL->onestep(directionL, SINGLE);
			steps2TakeL -= 1;
			leftStepsTaken++;
		}
		if (intStpR == 1){
			motorR->onestep(directionR, SINGLE);
			steps2TakeR -= 1;
			rightStepsTaken++;
		}
		float trapezeFactor = 1000;
		delay(trapezeFactor/speed);
	} // End for
	Serial.print("Left motor stepped: "); Serial.print(leftStepsTaken); Serial.println("full steps.");
	Serial.print("Right motor stepped: "); Serial.print(rightStepsTaken); Serial.println("full steps.");
	
	// microstep correction:
	float stepErrorL = stepsL-leftStepsTaken;
	float stepErrorR = stepsR-rightStepsTaken;
	Serial.print("Left motor error: "); Serial.print(stepErrorL); Serial.println(" steps.");
	Serial.print("Right motor error: "); Serial.print(stepErrorR); Serial.println(" steps.");
	
	return true;
}


int Actuators::moveLeft(int steps){ // only for debugging
	motorL->step(steps, FORWARD, SINGLE);
}

bool Actuators::calibration(){
	Serial.println("Calibrating...");
	angleL = START_ANGLE_LEFT;
	angleR = START_ANGLE_RIGHT;
	Serial.println("Calibration done!");
}

bool Actuators::setMagnet(bool status){

  return true;
}



bool Actuators::directK(int thetaL, int thetaR){

  return true;
}


Point * Actuators::intersection(Circle c1, Circle c2, bool & outOfRange){
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
	double  xi1 = xc + (h/d)*(y2-y1); // x coordinate of the first intersection point
	double  yi1 = yc - (h/d)*(x2-x1); // y coordinate of the first intersection point

	double  xi2 = xc - (h/d)*(y2-y1); // y coordinate of the second intersection point
	double  yi2 = yc + (h/d)*(x2-x1); // y coordinate of the second intersection point

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

MotorAngles Actuators::inverse(Point p, bool &outOfRange){
	MotorAngles motorAngles; // This will store the results of the calculation (theta1, theta2)
	motorAngles.theta1 = 0;
	motorAngles.theta2 = 0;
	
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
		motorAngles.theta1 = 90; 
		motorAngles.theta2 = 90;
	}
	else{ // The position is in the work area, so we calcualte the theta angles with atan2(x_elbow,y_elbow)
	
	// we ahve two sides of a triangle, we find the angle between the two sides:
	motorAngles.theta1 = -(180/M_PI)*atan2(LeftArm.x+dist/2, LeftArm.y); 
	// we ahve two sides of a triangle, we find the angle between the two sides:
	motorAngles.theta2 = (180/M_PI)*atan2(RightArm.x-dist/2 , RightArm.y); 
	
	}	
	// Free dinamically allocated memory
	free(intersect_pointsL);
	free(intersect_pointsR);
	
	return motorAngles;
	
}


