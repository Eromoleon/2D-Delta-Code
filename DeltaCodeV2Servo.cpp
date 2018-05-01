/*
Delta robot arduino control code
2018-05-01
*/

#include <Arduino.h>
#include <cppQueue.h>
#include <Wire.h>
//#include <Adafruit_MotorShield.h>
#include <Servo.h>

// ##############################
// Global constants::

// ..geometry:

#define START_ANGLE_LEFT 90
#define START_ANGLE_RIGHT 90
#define PICK_HEIGHT 200
const double  dist = 50; // distance between the two motors
const double  l1 = 70;  // length of the base arms (between a motor and a forearm)
const double  l2 = 140; // length of the forearms (that connect with the endeffector)

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

class Circle{
  public:
    Circle(double , double , double );
    bool isInside(Point p);
    double  x;
    double  y;
    double  r;
};

Circle::Circle(double xC, double yC, double rV){
  x = xC;
  y = yC;
  r = rV;
}

bool Circle::isInside(Point p){
  return pow( p.x-x , 2 ) + pow( p.y-y, 2 ) < pow(r,2);
}

struct MotorAngles{
  double  theta1;
  double  theta2;
};

Servo servoL;
Servo servoR;


Circle clu(-82,-40,150);
Circle cl0(-25,0,200);
Circle cru(82,-40,150);
Circle cr0(25,0,200);

//bool isInSafetyRange(Point p);

class Actuators{
	public:
		Actuators(Servo* servL, Servo* servR, int magnetP);
		bool taskMgr(Command c);
		bool move(float x, float y);
		bool pickAndP(int xCoord = 0, int color = 0);
		bool moveJ(float thetaL, float thetaR);
		bool calibration();
		bool setMagnet(bool status);
		bool inverseK(int xCoord, int yCoord);
		bool directK(int thetaL, int thetaR);
		Point * intersection(Circle c1, Circle c2, bool & outOfRange);
		MotorAngles inverse(Point p, bool &outOfRange);

	private:
	// TODO 4 points that constrain the work area
		Servo*  motorL;
		Servo*  motorR;
		int magnetPin;
		bool outOfReach;
		float angleR = 0;
		float angleL = 0;
		float thetaRev = STEPS_REVOLUTION;
		float angFactor = 360 / thetaRev;

};

// Functions:
bool taskMgr(Command c);

// Global variables:

Queue commandQ(sizeof(Command), FIFO_SIZE, FIFO);
int magnetP = MAGNET_PIN;
MotorAngles angles;
Actuators actuators(&servoL, &servoR, magnetP);
bool outOfRange = false;
String inputString = "";         // a String to hold incoming data



void setup() {


	// initialize serial:
	Serial.begin(115200);
	// reserve 200 bytes for the inputString:
	inputString.reserve(200);


	// Calibration:<
	//actuators.calibration();
	// Debugging:

	/*
	//debug motion:
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
	*/
	/* Point p;
	p.x = 100; p.y = 300;
	MotorAngles angls = actuators.inverse(p, outOfRange);
	Serial.println("angles: ");
	Serial.println(angls.theta1);
	Serial.print( " ");
	Serial.println( angls.theta2); */

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
  return true;
}

bool Actuators::move(float thetaL, float thetaR){
	moveJ(thetaL, thetaR);
	return true;
}

bool Actuators::inverseK(int xCoord, int yCoord){
	MotorAngles angles;
	Point p;
	p.x = xCoord;
	p.y = yCoord;
	angles = inverse(p, outOfRange);
	Serial.print("theta1: ");Serial.println(angles.theta1);
	Serial.print("theta2: ");Serial.println(angles.theta2);
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
    return true;
}

void serialEvent() {
    //Command structure: Number;Number;Number\n

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
    } // Seruak,avaukabke
}

Actuators::Actuators(Servo *servL, Servo *servR, int magnetP){
	motorL = servL;
	motorR = servR;
	magnetPin = magnetP;
}


// TODO: correct inaccuracy by using half- and microstepping at the end of the movement
bool Actuators::moveJ(float thetaL, float thetaR){ // Quasi-simultaneous operation of the two motors (Both motors stop at the same time) with trapez profile
	thetaL *= -1;
	return true;
}


bool Actuators::calibration(){
	Serial.println("Calibrating...");
	angleL = START_ANGLE_LEFT;
	angleR = START_ANGLE_RIGHT;
	Serial.println("Calibration done!");
    return true;
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
		outOfRange = true;
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
	// outputPoints [0] will have the greater y coordinate
	return outputPoints;
}

MotorAngles Actuators::inverse(Point p, bool &outOfRange){
	MotorAngles motorAngles; // This will store the results of the calculation (theta1, theta2)
	motorAngles.theta1 = angleL;
	motorAngles.theta2 = angleR;

	Circle cBaseL((-1)*dist/2, 0, l1 );
    Circle cBaseR(dist/2,0,l1);
    Circle cEnd(p.x, p.y,l2);

  // Left base circle: center is the left motor axis, radius is l1
	//cBaseL.x = (-1)*dist/2;
	//cBaseL.y = 0;
	//cBaseL.r = l1;

	// Right base circle: center is the right motor axis, radius is l1
//	cBaseR.x = dist/2;
//	cBaseR.y = 0;
//	cBaseR.r = l1;



	// End effector circle: center is the end effector position (p.x, p.y), radius is l2
	//cEnd.x = p.x;
	//cEnd.y = p.y;
//	cEnd.r = l2;

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
		Serial.println("out of Range!");
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
