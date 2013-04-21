#include "antenna_math.h"
#include "constants.h"


// USED FOR OFFLINE COMPILATION IN C (not robotC)
// #include <math.h>
// #include "Mymath.h"
// #include <stdio.h>

// Calculate two vectors:
// a2_from_a1: Vector [x y z] in ant_1 coordinates that points directly at ant_2
// a1_from_a2: Vector [x y z] in ant_2 coordinates that points directly at ant_1
static void relativeVectors( const fmat34 a1_mat, const fmat34 v1_mat, const fmat34 a2_mat,
							 const fmat34 v2_mat, fvec3 a1_from_a2, fvec3 a2_from_a1,
							 const fvec3 a1_pol, fvec3 a1_pol_in_a2_vec)
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
	
	// calculate antenna 1 polarization vector in antenna 3's coordinate system
	a1_pol_in_a2_vec[0] = t3_mat[0][0] * a1_pol[0] + t3_mat[0][1] * a1_pol[1] + t3_mat[0][2] * a1_pol[2];
	a1_pol_in_a2_vec[1] = t3_mat[1][0] * a1_pol[0] + t3_mat[1][1] * a1_pol[1] + t3_mat[1][2] * a1_pol[2];
	a1_pol_in_a2_vec[2] = t3_mat[2][0] * a1_pol[0] + t3_mat[2][1] * a1_pol[1] + t3_mat[2][2] * a1_pol[2];
	
}

// Return nearest neighbor point value in a sphereical coordinate system
static float antennaGain( const fmatdata gain_data, const fvec3 polar_vec )
{
	int elevation, azimuth;
	
	elevation = nearestGapDeg( polar_vec[1], GAP );
	azimuth = nearestGapDeg( polar_vec[2], GAP );
	
	return gain_data[elevation][azimuth];
}

//	returns signal strength, where 0 = perfect strength (zero distance)
float signalStrength( const fvec3 a1_pos_vec, const fvec3 v1_pos_vec, const fvec3 a2_pos_vec, 
				 	  const fvec3 v2_pos_vec, const ivec3 a1_rot_vec, const ivec3 v1_rot_vec,
				  	  const ivec3 a2_rot_vec, const ivec3 v2_rot_vec, const fvec3 a1_pol_vec,
					  const float a1_axial_ratio, const fvec3 a2_pol_vec, const float a2_axial_ratio,
					  const fmatdata a1_gain_data, const fmatdata a2_gain_data,
					  const float a1_wavelength, const float a1_t_power, const float a2_r_sensitivity,
					  const float a1_reflect_coef, const float a2_reflect_coef ) 
{	
	// build matrix representations
	float a1_mat[3][4], v1_mat[3][4], a2_mat[3][4], v2_mat[3][4];
	buildMat(a1_pos_vec, a1_rot_vec, a1_mat);
	buildMat(v1_pos_vec, v1_rot_vec, v1_mat);
	buildMat(a2_pos_vec, a2_rot_vec, a2_mat);
	buildMat(v2_pos_vec, v2_rot_vec, v2_mat);
	
	// calculate relative vector directions
	float a1_from_a2_vec[3], a2_from_a1_vec[3];
	relativeVectors(a1_mat, v1_mat, a2_mat, v2_mat, a1_from_a2_vec, a2_from_a1_vec, a1_pol_vec, a1_pol_in_a2_vec);
	
	// convert to polar coordinates
	float a1_from_a2_pol_vec[3], a2_from_a1_pol_vec[3];
	cartToSpher(a1_from_a2_vec, a1_from_a2_pol_vec);
	cartToSpher(a2_from_a1_vec, a2_from_a1_pol_vec);	
	
	// find antenna gain
	float t_gain, r_gain;
	t_gain = antennaGain(a1_gain_data, a2_from_a1_pol_vec);
	r_gain = antennaGain(a2_gain_data, a1_from_a2_pol_vec);
	
	// find polarization loss
	float pol_loss;
	pol_loss = PLF(a1_pol_in_a2_vec, a1_axial_ratio, a2_pol_vec, a2_axial_ratio)
	
	// calculate max distance where signal would be received
	float max_distance;	
	max_distance =	(sqrt(t_gain * r_gain * a1_t_power) * a1_wavelength) / 
					(sqrt(LOSS_FACTOR * a2_r_sensitivity) * 4 * pi());
	
	
	float distance, power_received;
	distance = sqrt(a1_from_a2_vec[0] * a1_from_a2_vec[0] + a1_from_a2_vec[1] * a1_from_a2_vec[1] +
					a1_from_a2_vec[2] * a1_from_a2_vec[2]);
	power_received = a1_t_power * t_gain * r_gain * (1 - a1_reflect_coef) * (1 - a1_reflect_coef) * pol_loss *
					 (a1_wavelength / (4 * pi() * distance));
				
	// first coordinate of polar vector is the distance.
	// 0 stregth: distance = max distance. 1 strength: distance = 0
	return power_received / a2_r_sensitivity;
}

//	returns signal strength, where 0 = extent of reception, and 1 = zero distance
int isConnected( const fvec3 a1_pos_vec, const fvec3 v1_pos_vec, const fvec3 a2_pos_vec, 
				 const fvec3 v2_pos_vec, const ivec3 a1_rot_vec, const ivec3 v1_rot_vec,
				 const ivec3 a2_rot_vec, const ivec3 v2_rot_vec, const fvec3 a1_pol_vec,
				 const float a1_axial_ratio, const fvec3 a2_pol_vec, const float a2_axial_ratio,
				 const fmatdata a1_gain_data, const fmatdata a2_gain_data,
				 const float a1_wavelength, const float a1_t_power, const float a2_r_sensitivity,
				 const float a1_reflect_coef, const float a2_reflect_coef )
{
	float signal_strength;
	signal_strength = signalStrength(a1_pos_vec, v1_pos_vec, a2_pos_vec, v2_pos_vec, a1_rot_vec, 
									 v1_rot_vec, a2_rot_vec, v2_rot_vec, a1_pol_vec, a1_axial_ratio,
									 a2_pol_vec, a2_axial_ratio, a1_gain_data, a2_gain_data,
					 				 a1_wavelength, a1_t_power, a2_r_sensitivity, a1_reflect_coef,
									 a2_reflect_coef);
	
	return signal_strength > 1;
}
/*
void antennaRead( const char file_name[], float out_gain[STEPS][STEPS], float pol_vec[3],
				  float *axial_ratio, float *t_wavelength, float *t_power, float *r_sens, float *reflect_coef )
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
		
		fread(&temp, 4, 1, f);
		*reflect_coef = (temp - 1 / temp + 1)	// temp is VSWR from antenna datasheet
		
		//fread(&temp, 4, 1, f);
		//	need to put something in here to read the polarization information with which we calculate PLF	
		
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
*/

void antennaRead( const char file_name[], float t_wavelength, float t_power, float r_sens,
				  float reflect_coef, fvec3 pol_vec, float axial_ratio,
				  fmatdata out_gain )
{
	TFileHandle file_handle;	// create a file handle variable 'myFileHandle'
	TFileIOResult io_result;	// create an IO result variable 'IOResult'
	int file_size;

	OpenRead(file_handle, io_result, file_name, file_size);    // open for read, size is 'myFileSize'
	ReadFloat(file_handle, io_result, t_wavelength);
	ReadFloat(file_handle, io_result, t_power);
	ReadFloat(file_handle, io_result, r_sens);
	
	float temp;
	ReadFloat(file_handle, io_result, temp);
	reflect_coef = (temp - 1 / temp + 1)	// temp is VSWR from antenna datasheet
	
	ReadFloat(file_handle, io_result, pol_vec[0]);
	ReadFloat(file_handle, io_result, pol_vec[1]);
	ReadFloat(file_handle, io_result, pol_vec[2]);
	ReadFloat(file_handle, io_result, axial_ratio);
	
	for(int i = 0; i < STEPS; i++)
	{
		for(int j = 0; j < STEPS; j++)
		{
			ReadFloat(file_handle, io_result, out_gain[i][j]);
		}
	}
	
}


static float PLF( const fvec3 a1_pol_in_a2_vec, const float a1_axial_ratio,
		   const fvec3 a2_pol_vec, const float a2_axial_ratio )
{
	// Poincaire Sphere representations:
	
	float a1_latitude, a2_latitude;
	
	if (a1_axial_ratio == 0)	{a1_latitude = 0;}
	else 						{a1_latitude = 2 * atan(1/a1_axial_ratio);}
	
	if (a2_axial_ratio == 0)	{a2+latitude = 0;}
	else						{a2_latitude = 2 * atan(1/a2_axial_ratio);}
	
	float temp, longitude;
	temp = (a1_pol_in_a2_vec[0] * a2_pol_vec[0] + 
			a1_pol_in_a2_vec[1] * a2_pol_vec[1] +
			a1_pol_in_a2_vec[2] * a2_pol_vec[2])
	
	longitude = 2 * acos(temp);
	
	float great_circle;
	great_circle = acos(cos(a1_latitude - a2_latitude) * cos(longitude));
	
	temp = cos(great_circle / 2);
	return temp * temp;
}
