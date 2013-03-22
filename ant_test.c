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
	
	printf("\nwavelength: ");
	printf("%f", t_wavelength);
	printf("\ntransmission power: ");
	printf("%f", t_power);
	printf("\nreceiving sensitivity: ");
	printf("%f\n", r_sens);
	
	float sig_strength;
	float a1_pos_vec[3] = {0, 0, 0};
	float v1_pos_vec[3] = {1, 0, 0};
	float a2_pos_vec[3] = {0, 0, 0};
	float v2_pos_vec[3] = {-1, 4, 0};
	int a1_rot_vec[3] = {0, 110, 0};
	int v1_rot_vec[3] = {90, 90, 23};
	int a2_rot_vec[3] = {0, 0, 0};
	int v2_rot_vec[3] = {0, 0, 0};
	float a1_gain_data[STEPS][STEPS];
	float a2_gain_data[STEPS][STEPS];
	float a1_wavelength;
	float a1_t_power;
	float a2_r_sensitivity;
	
	antennaRead( "Yagi9dBi.dat", a1_gain_data, &a1_wavelength, &a1_t_power, &a2_r_sensitivity );
	antennaRead( "Yagi9dBi.dat", a2_gain_data, &a1_wavelength, &a1_t_power, &a2_r_sensitivity );
	
	sig_strength = signalStrength( a1_pos_vec, v1_pos_vec, a2_pos_vec, v2_pos_vec, 
								   a1_rot_vec, v1_rot_vec, a2_rot_vec, v2_rot_vec, 
								   a1_gain_data, a2_gain_data, a1_wavelength, a1_t_power, a2_r_sensitivity);
	
	printf("\nsignal strength: ");
	printf("%f\n", sig_strength);	
	
	return 0;
}