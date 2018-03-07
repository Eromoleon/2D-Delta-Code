/*
Delta robot arduino control code
2018-03-07
*/

// includes
#include <Arduino.h>
#include <cppQueue.h>
#include <Adafruit_MotorShield.h>

// Global declarations:

#define COMMAND_LENGTH 10
#define FIFO_SIZE 20
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


const double  dist = 100; // distance between the two motors
const double  l1 = 150;  // length of the base arms (between a motor and a forearm)
const double  l2 = 300; // length of the forearms (that connect with the endeffector)
String inputString = "";         // a String to hold incoming data
Queue	commandQ(sizeof(Command), FIFO_SIZE, FIFO);


// Functions:
bool moveJ(int thetaL, int thetaR);
bool elektroM(bool status);
bool inverseK(Command);
bool directK(Command);
bool pickAndP(Command);
Point * intersection(Circle c1, Circle c2, bool & outOfRange); // calculates the intersection points between two circles.
															   // if there is no intersection, outOfRange is set to true.
MotorAngles inverse(Point p, bool &outOfRange);	// calculates base arm angles theta1 and theta2 from on the endpoint position.
												// if it is unreachable by the robot, outOfRange is set to true.
MotorAngles angles;
void setup() {
  // initialize serial:
  Serial.begin(115200);
  // reserve 200 bytes for the inputString:
  inputString.reserve(200);
  Serial.println("Setup complete");
  Point p;
  p.x = 100; p.y = 300;
  MotorAngles angls = inverse(p, outOfRange);
  Serial.println("angles: ");
  Serial.println(angls.theta1);
  Serial.print( " ");
  Serial.println( angls.theta2);

} // __End setup__

void loop() {

	if(commandQ.isEmpty())
	{
		Serial.println("Command queue empty, waiting for command...");
		delay(10000);
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
} // End SerialEvent

bool moveJ(int thetaL, int thetaR){

	return true;
}

bool elektroM(bool status){

	return true;
}

bool inverseK(Command com){

	return true;
}

bool directK(Command co){

	return true;
}

bool pickAndP(Command comm){

	return true;
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

MotorAngles inverse(Point p, bool &outOfRange){

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

