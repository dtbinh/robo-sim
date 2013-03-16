#include <stdio.h>
#include <stdlib.h>
#include "Mymath.h"
#include "antenna_math.h"
const float A_in[3][4]={{1,2,3,10},{4,5,6,20},{7,8,9,30}};
float A_T[3][4]={0}; 
	int main(int argc, char *argv[]) 
{	

	// Original 3X4 Matrix //
	printf("\nOriginal Matrix:\n ");
	printf("\t%f\t %f\t %f\t %f\t\n",A_in[0][0],A_in[0][1],A_in[0][2],A_in[0][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_in[1][0],A_in[1][1],A_in[1][2],A_in[1][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_in[2][0],A_in[2][1],A_in[2][2],A_in[2][3]);
	// END OF SECTION OF CODE // 
	
	// Orignal 3X4 Matrix Inverted //
	matInverse(A_in, A_T);
	printf("\nInput Matrix Inverted:\n ");
	printf("\b\t%f\t %f\t %f\t %f\t\n",A_T[0][0],A_T[0][1],A_T[0][2],A_T[0][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_T[1][0],A_T[1][1],A_T[1][2],A_T[1][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_T[2][0],A_T[2][1],A_T[2][2],A_T[2][3]);
	// END OF SECTION OF CODE // 
	
	// Matrix Multiplication //
	const float A_mult[3][4]={{1,0,0,10},{0,1,0,20},{0,0,1,30}}, B_mult[3][4]={{1,1,1,40},{1,1,1,30},{1,1,1,20}};
	float C_mult[3][4]={0};
	matMatMult(A_mult,B_mult,C_mult);
	printf("\nInput Matrices Multiplied:\n ");
	printf("\b\t%f\t %f\t %f\t %f\t\n",C_mult[0][0],C_mult[0][1],C_mult[0][2],C_mult[0][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",C_mult[1][0],C_mult[1][1],C_mult[1][2],C_mult[1][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",C_mult[2][0],C_mult[2][1],C_mult[2][2],C_mult[2][3]);
	// END OF SECTION OF CODE // 
	
	// Euler2Rotation //
	float Rot_3X3[3][3]={{0,0,0},{0,0,0},{0,0,0}};
	const float *Rot_1X3[3]={0,0,0};
	euler2Rot(Rot_1X3,Rot_3X3);
	printf("\nEuler2Rotation:\n ");
	printf("\b\t%f\t %f\t %f\n",Rot_3X3[0][0],Rot_3X3[0][1],Rot_3X3[0][2]);
	printf("\t%f\t %f\t %f\n",Rot_3X3[1][0],Rot_3X3[1][1],Rot_3X3[1][2]);
	printf("\t%f\t %f\t %f\n",Rot_3X3[2][0],Rot_3X3[2][1],Rot_3X3[2][2]);
	//END OF SECTION OF CODE // 
	
	// Cartesian to Spherical Coordinates //
	const float cart[3]={sqrt(2.0)/2.0,sqrt(2.0)/2.0,1.0};
	float polar[3]={0};
	cartToSpher(cart, polar);
	printf("\nCartesian2Spherical:\n");
	printf("\bRadius: %f\n",polar[0]);
	printf("Elevation(theta): %f\n",polar[1]);	
	printf("Azimuth(phi): %f\n",polar[2]);
	// END OF SECTION OF CODE // 
	
	// MatrixBuilder //
	float position[3]={A_in[0][3],A_in[1][3],A_in[2][3]};
	buildMat(position, Rot_1X3, A_T);
	printf("\nMatrix Builder: \n ");
	printf("\b\t%f\t %f\t %f\t %f\t\n",A_T[0][0],A_T[0][1],A_T[0][2],A_T[0][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_T[1][0],A_T[1][1],A_T[1][2],A_T[1][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_T[2][0],A_T[2][1],A_T[2][2],A_T[2][3]);
	// END OF SECTION OF CODE // 
	
	// nearestGapDeg //
	float deg =1;
	int gap = 1;
	nearestGapDeg(deg, gap);
	printf("\nnearestGapDeg(%f, %d): %d\n ",deg, gap, nearestGapDeg(deg, gap));
	// END OF SECTION OF CODE // 
	
	// Trig Conversion Implementation //
	float degrees=360,value=sqrt(2)/2; 
	printf("\ncosDegrees(%f): ",degrees);
	printf("%f", cosDegrees(degrees));
	printf("\nsinDegrees(%f): ",degrees);
	printf("%f", sinDegrees(degrees));
	printf("\ntanDegrees(%f): ",degrees);
	printf("%f", tanDegrees(degrees));
	printf("\nacosDegrees(%f): ",value);
	printf("%f", acosDegrees(value));
	printf("\nasinDegrees(%f): ",value);
	printf("%f", asinDegrees(value));
	printf("\natanDegrees(%f): ",value);
	printf("%f", atanDegrees(value));
	// END OF SECTION OF CODE // 
	
}
