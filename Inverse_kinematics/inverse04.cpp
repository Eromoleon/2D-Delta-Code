#include <iostream>
#include <stdlib.h>   
#include <math.h>

using namespace std;
//
// Constants:
const double  dist = 100;
const double  l1 = 150;
const double  l2 = 300;

//
// Typedefs:
struct Point{
	double  x;
	double  y;
};

struct Circle{
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
Point * intersection(Circle c1, Circle c2);
ServoAngles inverse(Point p, bool &outOfRange);

int main(){
	
	Point endPoint;
	cout<<"Enter point x: ";
	cin  >> endPoint.x; 
	
	cout<<"Enter point y: ";
	cin>> endPoint.y;
	bool outOfRange = false;
	ServoAngles angles = inverse(endPoint, outOfRange);
	if(outOfRange == false){
		cout << "Servo angles:" << endl  << " theta 1 = " << (180/3.14)*angles.theta1 << endl<<" theta 2 = " <<  (180/3.14)*angles.theta2 << endl;
	}
	else{
		cout << "Out of range!" << endl;
	}
	return 0;
}


//
// Function definitions:
Point * intersection(Circle c1, Circle c2){
	//double  * output = (double *)malloc(4*sizeof(double ));
	Point * outputPoints = (Point*)malloc(2*sizeof(Point));
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
		// cout<<"No intersections"<<endl;
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
	double  xi1 = xc + (h/d)*(y2-y1);
	double  yi1 = yc - (h/d)*(x2-x1);
		
	double  xi2 = xc - (h/d)*(y2-y1);
	double  yi2 = yc + (h/d)*(x2-x1);
	
	//pre-select intersection:
	
	if (abs(xi1)>abs(xi2)){
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
	outOfRange = false;
	ServoAngles servoAngles;
	servoAngles.theta1 = 0;
	servoAngles.theta2 = 0;
	Circle cBaseL, cBaseR, cEnd;
	cBaseR.x = dist/2;
	cBaseR.y = 0;
	cBaseR.r = l1;
	
	cBaseL.x = (-1)*dist/2;
	cBaseL.y = 0;
	cBaseL.r = l1;
	
	cEnd.x = p.x;
	cEnd.y = p.y;
	cEnd.r = l2;
	
	// find intersection left:
	Point * intersect_pointsL;
	intersect_pointsL = intersection(cBaseL, cEnd);
	
	// select intersection:
	Point LeftArm;
	if(intersect_pointsL[0].x < intersect_pointsL[1].x){
		LeftArm = intersect_pointsL[0];
	}
	else{
		LeftArm = intersect_pointsL[1];	
	}
	cout<< "LeftIntersect: "<<LeftArm.x<<", "<<LeftArm.y<<endl;
	
	// find intersection right:
	Point * intersect_pointsR;
	intersect_pointsR = intersection(cBaseR, cEnd);
	
	//select intersection
	Point RightArm;
	if(intersect_pointsR[0].x > intersect_pointsR[1].x){
		RightArm = intersect_pointsR[0];
	}
	else{
		RightArm = intersect_pointsR[1];
	}
	cout<< "RightIntersect: "<<RightArm.x<<", "<<RightArm.y<<endl;
	
	if(RightArm.y == -1||LeftArm.y==-1){
		outOfRange = true;
		// Set the angles to a safe value
		servoAngles.theta1 = 90; 
		servoAngles.theta2 = 90;
	}
	else{
	servoAngles.theta1 = -atan2(LeftArm.x+dist/2, LeftArm.y); // - M_PI/2;  
	servoAngles.theta2 = atan2(RightArm.x-dist/2 , RightArm.y); //M_PI/2 - 
	
	}	
	free(intersect_pointsL);
	free(intersect_pointsR);
	return servoAngles;
}
