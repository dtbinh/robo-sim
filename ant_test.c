#include <stdio.h>
#include <stdlib.h>
#include "Mymath.h"
#include "antenna.h"
#include "antenna_math.h"

const float A_in[3][4]={{1,2,3,10},{4,5,6,20},{7,8,9,30}};
float A_T[3][4]={{0}};

int main (int argc, char *argv[])
{
	// Original 3X4 Matrix
	printf("\nOriginal Matrix:\n ");
	printf("\t%f\t %f\t %f\t %f\t\n",A_in[0][0],A_in[0][1],A_in[0][2],A_in[0][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_in[1][0],A_in[1][1],A_in[1][2],A_in[1][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_in[2][0],A_in[2][1],A_in[2][2],A_in[2][3]);
	// END OF SECTION OF CODE // 
	
	// Orignal 3X4 Matrix Inverted
	matInverse(A_in, A_T);
	printf("\nInput Matrix Inverted:\n ");
	printf("\b\t%f\t %f\t %f\t %f\t\n",A_T[0][0],A_T[0][1],A_T[0][2],A_T[0][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_T[1][0],A_T[1][1],A_T[1][2],A_T[1][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",A_T[2][0],A_T[2][1],A_T[2][2],A_T[2][3]);
	// END OF SECTION OF CODE // 
	
	// Matrix Multiplication
	const float A_mult[3][4]={{1,0,0,10},{0,1,0,20},{0,0,1,30}}, B_mult[3][4]={{1,1,1,40},{1,1,1,30},{1,1,1,20}};
	float C_mult[3][4]={{0}};
	matMatMult(A_mult,B_mult,C_mult);
	printf("\nInput Matrices Multiplied:\n ");
	printf("\b\t%f\t %f\t %f\t %f\t\n",C_mult[0][0],C_mult[0][1],C_mult[0][2],C_mult[0][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",C_mult[1][0],C_mult[1][1],C_mult[1][2],C_mult[1][3]);
	printf("\t%f\t %f\t %f\t %f\t\n",C_mult[2][0],C_mult[2][1],C_mult[2][2],C_mult[2][3]);
	// END OF SECTION OF CODE //
	
	float out_gain[STEPS][STEPS];
	float t_wavelength;
	float t_power;
	float r_sens;
	antennaRead( "Yagi9dBi.dat", out_gain, &t_wavelength, &t_power, &r_sens );
	
	
	printf("\nout_gain: \n");
	
	int i;
	int j;
	
	for( i = 0; i < STEPS; i++) {
	    for( j = 0; j < STEPS; j++) {
	        printf("%f ", out_gain[i][j]);
	    }
	    printf("\n");
	}
	
	
	printf("\nwavelength: ");
	printf("%f", t_wavelength);
	printf("\ntransmission power: ");
	printf("%f", t_power);
	printf("\nreceiving sensitivity: ");
	printf("%f\n", r_sens);
	
	return 0;
}