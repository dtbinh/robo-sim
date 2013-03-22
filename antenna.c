#include "antenna_math.h"
#include "constants.h"
#include <math.h>
#include "Mymath.h"
#include <stdio.h>

// Calculate two vectors:
// a2_from_a1: Vector [x y z] in ant_1 coordinates that points directly at ant_2
// a1_from_a2: Vector [x y z] in ant_2 coordinates that points directly at ant_1
static void relativeVectors( const float a1_mat[3][4], const float v1_mat[3][4], const float a2_mat[3][4],
							 const float v2_mat[3][4], float a1_from_a2[3], float a2_from_a1[3])
{
	float t1_mat[3][4];
	float t2_mat[3][4];
	float t3_mat[3][4];
	
	// calculating i_ant_1 * i_robot_1 * robot_2 * ant_2 yields ant_2 from the
	// coordinate system of ant_1
	
	matInverse(a1_mat, t1_mat);
	matInverse(v1_mat, t2_mat);
	
	matMatMult(t1_mat, t2_mat, t3_mat);
	matMatMult(t3_mat, v2_mat, t1_mat);
	matMatMult(t1_mat, a2_mat, t3_mat);		// t3_Mat is now ant_2 from ant_1
	
	a2_from_a1[0] = t3_mat[0][3];
	a2_from_a1[1] = t3_mat[1][3];
	a2_from_a1[2] = t3_mat[2][3];
	
	matInverse(t3_mat, t1_mat);				// inverting to get ant_1 from ant_2's perspective
	
	a1_from_a2[0] = t1_mat[0][3];
	a1_from_a2[1] = t1_mat[1][3];
	a1_from_a2[2] = t1_mat[2][3];
}

// Return nearest neighbor point value in a sphereical coordinate system
static float antennaGain( const float gain_data[STEPS][STEPS], const float polar_vec[3] )
{
	int elevation, azimuth;
	
	elevation = nearestGapDeg( polar_vec[1], GAP );
	azimuth = nearestGapDeg( polar_vec[2], GAP );
	
	return gain_data[elevation][azimuth];
}

//	returns signal strength, where 0 = perfect strength (zero distance)
float signalStrength( const float a1_pos_vec[3], const float v1_pos_vec[3], const float a2_pos_vec[3], 
				 	  const float v2_pos_vec[3], const int a1_rot_vec[3], const int v1_rot_vec[3],
				  	  const int a2_rot_vec[3], const int v2_rot_vec[3], const float a1_gain_data[STEPS][STEPS],
				  	  const float a2_gain_data[STEPS][STEPS], const float a1_wavelength, const float a1_t_power,
				  	  const float a2_r_sensitivity ) 
{	
	// build matrix representations
	float a1_mat[3][4], v1_mat[3][4], a2_mat[3][4], v2_mat[3][4];
	buildMat(a1_pos_vec, a1_rot_vec, a1_mat);
	buildMat(v1_pos_vec, v1_rot_vec, v1_mat);
	buildMat(a2_pos_vec, a2_rot_vec, a2_mat);
	buildMat(v2_pos_vec, v2_rot_vec, v2_mat);
	
	// calculate relative vector directions
	float a1_from_a2_vec[3], a2_from_a1_vec[3];
	relativeVectors(a1_mat, v1_mat, a2_mat, v2_mat, a1_from_a2_vec, a2_from_a1_vec);
	
	// convert to polar coordinates
	float a1_from_a2_pol_vec[3], a2_from_a1_pol_vec[3];
	cartToSpher(a1_from_a2_vec, a1_from_a2_pol_vec);
	cartToSpher(a2_from_a1_vec, a2_from_a1_pol_vec);	
	
	// find antenna gain
	float t_gain, r_gain;
	t_gain = antennaGain(a1_gain_data, a2_from_a1_pol_vec);
	r_gain = antennaGain(a2_gain_data, a1_from_a2_pol_vec);
	
	// calculate max distance where signal would be received
	float max_distance;	
	max_distance =	(sqrt(t_gain * r_gain * a1_t_power) * a1_wavelength) / 
					(sqrt(LOSS_FACTOR * a2_r_sensitivity) * 4 * pi());
	
	// first coordinate of polar vector is the distance.
	// 0 stregth: distance = max distance. 1 strength: distance = 0
	return -(a2_from_a1_pol_vec[0] / max_distance) + 1;
}

//	returns signal strength, where 0 = extent of reception, and 1 = zero distance
int isConnected( const float a1_pos_vec[3], const float v1_pos_vec[3], const float a2_pos_vec[3], 
			  	 const float v2_pos_vec[3], const int a1_rot_vec[3], const int v1_rot_vec[3],
				 const int a2_rot_vec[3], const int v2_rot_vec[3], const float a1_gain_data[STEPS][STEPS],
				 const float a2_gain_data[STEPS][STEPS], const float a1_wavelength, const float a1_t_power,
				 const float a2_r_sensitivity )
{
	float signal_strength;
	signal_strength = signalStrength(a1_pos_vec, v1_pos_vec, a2_pos_vec, v2_pos_vec,
	 			   					 a1_rot_vec, v1_rot_vec, a2_rot_vec, v2_rot_vec,
				   					 a1_gain_data, a2_gain_data, 
  									 a1_wavelength, a1_t_power, a2_r_sensitivity);
	
	return signal_strength > 0;
}

void antennaRead( const char file_name[], float out_gain[STEPS][STEPS], float *t_wavelength, float *t_power, float *r_sens )
{
	FILE *f;
	int i;
	int j;
	float temp;
	
	if (f = fopen(file_name, "rb"))
	{
		fread(&temp, 4, 1, f);
		*t_wavelength = temp;
		
		fread(&temp, 4, 1, f);
		*t_power = temp;
		
		fread(&temp, 4, 1, f);
		*r_sens = temp;
		
		for( i = 0; i < STEPS; i++)
		{
			for( j = 0; j < STEPS; j++)
			{
				fread(&temp, 4, 1, f);
				out_gain[i][j] = temp;
			}
		}
		
		fclose(f);
	}
	
}
/*
void antennaRead( const char file_name[], float out_gain[STEPS][STEPS], float t_wavelength, float t_power, float r_sens )
{
	TFileHandle file_handle;	// create a file handle variable 'myFileHandle'
	TFileIOResult io_result;	// create an IO result variable 'IOResult'
	int file_size;

	OpenRead(file_handle, io_result, file_name, file_size);  // open for read: "myFile.txt",
	                                                           // storing its size in 'myFileSize'
	ReadFloat(file_handle, io_result, t_wavelength);
	ReadFloat(file_handle, io_result, t_power);
	ReadFloat(file_handle, io_result, r_sens);
	for(int i = 0; i < STEPS; i++)
	{
		for(int j = 0; j < STEPS; j++)
		{
			ReadFloat(file_handle, io_result, out_gain[i][j]);
		}
	}
	
}
*/
