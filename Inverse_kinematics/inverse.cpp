#include <iostream>
#include <stdlib.h>   
#include <math.h>

using namespace std;
double * intersection(double input[6]);

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

int main(){
	
	double input_data[6];
	double * output_data;
	
	for(int i=0; i<sizeof(input_data)/sizeof(double); i++){
		cout<<"Enter input data number "<<i<<": ";
		cin>>input_data[i];
	}
	
	output_data = intersection(input_data);
	
	cout<<"xi1, yi1: "<<output_data[0]<<" , "<<output_data[1]<<endl;
	cout<<"xi2, yi2: "<<output_data[2]<<" , "<<output_data[3]<<endl;
	free(output_data);
	return 0;
}
/*
void hour_payload_add(int entries , double array[], double (&returnArray)[SIZE])
{
  // returnArray will be updated as it's external to function.
}
*/
double * intersection(double input[6]){
	double * output = (double*)malloc(4*sizeof(double));
	double x1 = input[0];
	double y1 = input[1];
	double r1 = input[2];
	double x2 = input[3];
	double y2 = input[4];
	double r2 = input[5];
	// P1, P2: center points, I1, I2: intersection points
	// L: line connecting the center points
	double centerdx = x1 - x2;
	double centerdy = y1 - y2;
	double d = sqrt(centerdx * centerdx + centerdy * centerdy);
	if (!(abs(r1 - r2) <= d && d <= r1 + r2)) { // no intersection
		cout<<"No intersections"<<endl;
		return output;
		
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
	output[0] = xi1;
	output[1] = yi1;
	output[2] = xi2;
	output[3] = yi2;
	return output;
}
