/*
Delta robot arduino control code
2018-03-07
*/

// includes
#include <cppQueue.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>





// Global declarations:
#define START_ANGLE_LEFT 0;
#define START_ANGLE_RIGHT 0;
#define MAGNET_PIN 1
#define COMMAND_LENGTH 10
#define FIFO_SIZE 20
#define STEPS_REVOLUTION 200

const double  dist = 100; // distance between the two motors
const double  l1 = 150;  // length of the base arms (between a motor and a forearm)
const double  l2 = 300; // length of the forearms (that connect with the endeffector)


bool outOfRange = false;
// Typedefs:


struct Command  {
  int determinant;
  int elements[COMMAND_LENGTH];
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
		bool move(float x, float y);
		bool moveJ(float thetaL, float thetaR);
		bool moveTrapez(float thetaL , float thetaR);
		bool calibration();
		bool setMagnet(bool status);
		bool inverseK(Command c);
		bool directK(Command c);
		bool pickAndP(Command c);
		int moveLeft(int steps);
		Point * intersection(Circle c1, Circle c2, bool & outOfRange);
		MotorAngles inverse(Point p, bool &outOfRange);
		
	private:
		Adafruit_StepperMotor*  motorL;
		Adafruit_StepperMotor*  motorR;
		int magnetPin;		
		bool outOfReach;
		float angleR = 0;
		float angleL = 0;
		
};



String inputString = "";         // a String to hold incoming data

Queue commandQ(sizeof(Command), FIFO_SIZE, FIFO);


int magnetP = MAGNET_PIN;
MotorAngles angles;
Actuators actuators(motL, motR, magnetP);

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
	
	motL->setSpeed(10);  // 10 rpm   
	
	actuators.moveJ(70, 100);
	actuators.moveJ(-70, -100);
	actuators.moveJ(100, 70);
	actuators.moveJ(-100, -70);
	
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
    switch(c.determinant){
		case 0:
			Serial.println("Normal operation, piece sorting");
			Serial.print("Right Left Forward?: ");
			Serial.println(c.elements[1]);
			Serial.print("X: ");
			Serial.println(c.elements[2]);
			Serial.println("Executing command...");
			delay(2000);
			Serial.println("\n");

			break;
		case 1:
			Serial.println("Direct Kinematics");
			Serial.print("thetaL: ");
			Serial.println(c.elements[1]);
			Serial.print("thetaR: ");
			Serial.println(c.elements[2]);
			Serial.println("Executing command...");
			delay(2000);
			Serial.println("\n");
			break;
		default:
			Serial.println("Undefined command type.");
			break;
    }
    Serial.println("#########################################################");


  }

    delay(10);
  //}
}
// __End loop__

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
        c.elements[j] = 0; // Initialize the command array
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
				c.elements[nCount] = (int)number;
				nCount++;
				numString = "";
			}
      }

      c.determinant = c.elements[0] ;
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

bool Actuators::move(float x, float y){
	


  return true;
}





bool Actuators::moveTrapez(float thetaL , float thetaR){
	
}

bool Actuators::moveJ(float stepsL, float stepsR){ // Quasi-simultaneous operation of the two motors (Both motors stop at the same time)
	int directionL = 1;
	int directionR = 1;
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
	float stepRatio = 0.0;
	float stepRatioR = 0.0;
	float steps2Take = 0.0;
	float steps2TakeR = 0.0;
	float minSteps = 0.0;
	if(stepsL > stepsR){
		stepRatio = stepsL / stepsR;
		stepRatioR = 1;
		minSteps = stepsR;
	}
	else {
		stepRatio  = 1;
		stepRatioR = stepsR / stepsL;
		minSteps = stepsL;
	}
	Serial.println("Stepping left motor with ");
	Serial.print(stepsL); Serial.println(" steps.");
	Serial.println("Stepping right motor with ");
	Serial.print(stepsR); Serial.println(" steps.");
	
	// Trapez profil paramteters: 
	
	int maxSpeed = 500; // RPM
	int startSpeed = 1;
	int t1R = int( round( stepsR / 3 ));
	int t1L = int( round( stepsL / 3 ));
	int speedR = 0;
	int speedL = 0;
	int speedIncrL = int(round((maxSpeed-startSpeed)/t1L));
	int speedIncrR = int(round((maxSpeed-startSpeed)/t1R));
	
	for(int i = 0; i < int(round(minSteps)); i++){
		
		steps2TakeR += stepRatioR;
		int integerStepR = int(round(steps2TakeR));
		
		for(int j=0; j<integerStepR; j++){
			if(rightStepsTaken < t1R && speedR <= maxSpeed ){
				speedR+=speedIncrR;
				motorR->setSpeed(speedR);
			
			}
			else if(rightStepsTaken > stepsR-t1R){
				speedR -= speedIncrR;
				motorR->setSpeed(speedR);
			}
			motorR->step(1,directionR,SINGLE);
			rightStepsTaken++;
		}
		//motorR->step(integerStepR,directionR,SINGLE);
		
		angleR +=integerStepR*1.8*dirR;
		Serial.print("Current angle R: "); Serial.println(angleR);
		
		steps2TakeR -= integerStepR;
		
		
		steps2Take += stepRatio;
		int integerStepL = int(round(steps2Take));
	
		for(int j=0; j<integerStepL; j++){
			if(leftStepsTaken < t1L && speedL <= maxSpeed ){
				speedL+=speedIncrL;
				motorL->setSpeed(speedL);
			
			}
			else if(leftStepsTaken > stepsL-t1L){
				speedL -= speedIncrL;
				motorL->setSpeed(speedL);
			}
			
			motorL->step(1,directionL,SINGLE);
			leftStepsTaken++;
		}
		//motorL->step(integerStepL, directionL, SINGLE);
		
		angleL +=integerStepL*1.8*dirL;
		Serial.print("Current angle L: "); Serial.println(angleL);
		Serial.println("");
		steps2Take -= integerStepL;
	
	} // End for
	Serial.print("Left motor stepped: "); Serial.print(leftStepsTaken); Serial.println(" steps.");
	Serial.print("Right motor stepped: "); Serial.print(rightStepsTaken); Serial.println(" steps.");
	return true;
}


int Actuators::moveLeft(int steps){ // only for debugging
	motorL->step(steps, FORWARD, SINGLE);
}

bool Actuators::calibration(){
	
	angleL = START_ANGLE_LEFT;
	angleR = START_ANGLE_RIGHT;
}

bool Actuators::setMagnet(bool status){

  return true;
}

bool Actuators::inverseK(Command c){

  return true;
}

bool Actuators::directK(Command c){

  return true;
}

bool Actuators::pickAndP(Command c){

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


