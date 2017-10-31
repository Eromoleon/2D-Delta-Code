#include <iostream>
#include <stdlib.h>   
#include <math.h>

using namespace std;
//
// Constants:
const int dist = 100;
const int l1 = 300;
const int l2 = 150;

//
// Typedefs:
struct Point{
	double x;
	double y;
};

struct Circle{
	double x;
	double y;
	double r;
};

struct ServoAngles{
	double theta1;
	double theta2;
};

//
// Function headers:
Point * intersection(Circle c1, Circle c2);
ServoAngles inverse(Point p);

int main(){
	
	double input_data[6];
	//double * output_data;
	
	for(int i=0; i<sizeof(input_data)/sizeof(double); i++){
		cout<<"Enter input data number "<<i<<": ";
		cin>>input_data[i];
	}
	Circle c1, c2;
	c1.x =input_data[0];
	c1.y =input_data[1];
	c1.r =input_data[2];
	c2.x =input_data[3];
	c2.y =input_data[4];
	c2.r =input_data[5];
	
	Point * output_points;
	output_points = intersection(c1, c2);
	cout<<"xi1, yi1: "<<output_points[0].x<<" , "<<output_points[0].y<<endl;
	cout<<"xi2, yi2: "<<output_points[1].x<<" , "<<output_points[1].y<<endl;
	/*
	cout<<"xi1, yi1: "<<output_data[0]<<" , "<<output_data[1]<<endl;
	cout<<"xi2, yi2: "<<output_data[2]<<" , "<<output_data[3]<<endl;
	*/
	free(output_points);
	
	
	
	//free(output_data);
	return 0;
}
/*
void hour_payload_add(int entries , double array[], double (&returnArray)[SIZE])
{
  // returnArray will be updated as it's external to function.
}
*/

//
// Function definitions:
Point * intersection(Circle c1, Circle c2){
	//double * output = (double*)malloc(4*sizeof(double));
	Point * outputPoints = (Point*)malloc(2*sizeof(Point));
	double x1 = c1.x;
	double y1 = c1.y;
	double r1 = c1.r;
	double x2 = c2.x;
	double y2 = c2.y;
	double r2 = c2.r;
	// P1, P2: center points, I1, I2: intersection points
	// L: line connecting the center points
	double centerdx = x1 - x2;
	double centerdy = y1 - y2;
	double d = sqrt(centerdx * centerdx + centerdy * centerdy);
	if (!(abs(r1 - r2) <= d && d <= r1 + r2)) { // no intersection
		cout<<"No intersections"<<endl;
		//return outputPoints;
		
	 // empty list of results
	}
		 // intersection(s) should exist
	double a = (r1*r1-r2*r2+d*d)/(2*d); // length of the orthogonal projection of P1_I1 vector to L (as well as P1I2 to L) 
	double h = sqrt(r1*r1-a*a); // length of the orthogonal complement of P1_I1 vector to L
	
	double xc = x1+(a/d)*(x2-x1); // Endpoint x of the orthogonal projection of P1_I1 vector to L
	double yc = y1+(a/d)*(y2-y1); // Endpoint y of the orthogonal projection of P1_I1 vector to L
	// I1, I2:
	double xi1 = xc + (h/d)*(y2-y1);
	double yi1 = yc - (h/d)*(x2-x1);
		
	double xi2 = xc - (h/d)*(y2-y1);
	double yi2 = yc + (h/d)*(x2-x1);
	
	outputPoints[0].x = xi1;	
	outputPoints[0].y = yi1;
	outputPoints[1].x = xi2;
	outputPoints[1].y = yi2;
	
	return outputPoints;
}

ServoAngles inverse(Point p){
	ServoAngles servoAngles;
	servoAngles.theta1 = 0;
	servoAngles.theta2 = 0;
	Circle cBaseL, cBaseR, cEnd;
	
	cBaseR.x = -dist/2;
	cBaseR.y = 0;
	cBaseR.r = l1;
	
	cBaseL.x = dist/2;
	cBaseL.y = 0;
	cBaseL.r = l1;
	
	cEnd.x = p.x;
	cEnd.y = p.y;
	cEnd.r = l2;
	
	Point * intersect_pointsL;
	intersect_pointsL = intersection(cBaseL, cEnd);
	Point LeftArm = intersect_pointsL[0];
	free(intersect_pointsL);
	
	Point * intersect_pointsR;
	intersect_pointsR = intersection(cBaseR, cEnd);
	Point RightArm = intersect_pointsR[0];
	free(intersect_pointsR);
	
	servoAngles.theta1 = atan2(LeftArm.y, LeftArm.x);
	servoAngles.theta2 = atan2(RightArm.y, RightArm.x);
	
	return servoAngles;
}
