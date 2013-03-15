#include "antenna_math.h"

// Calculate two vectors:
// 2a_from_1a: Vector [x y z] in ant_1 coordinates that points directly at ant_2
// 1a_from_2a: Vector [x y z] in ant_2 coordinates that points directly at ant_1
static void relativeVectors( const float 1a_mat[3][4], const float 1v_mat[3][4], const float 2a_mat[3][4],
							 const float 2v_mat[3][4], float 1a_from_2a[3], float 2a_from_1a[3])
{
	float[3][4] t1_mat, t2_mat, t3_mat;
	
	// calculating i_ant_1 * i_robot_1 * robot_2 * ant_2 yields ant_2 from the
	// coordinate system of ant_1
	
	MatInverse(1a_mat, t1_mat);
	MatInverse(1v_mat, t2_mat);
	
	MatMatMult(t1_mat, t2_mat, t3_mat);
	MatMatMult(t3_mat, 2v_mat, t1_mat);
	MatMatMult(t1_mat, 2a_mat, t3_mat);		// t3_Mat is now ant_2 from ant_1
	
	2a_from_1a[0] = t3_mat[0][3];
	2a_from_1a[1] = t3_mat[1][3];
	2a_from_1a[2] = t3_mat[2][3];
	
	MatInverse(t3_mat, t1_mat);				// inverting to get ant_1 from ant_2's perspective
	
	1a_from_2a[0] = t1_mat[0][3];
	1a_from_2a[1] = t1_mat[1][3];
	1a_from_2a[2] = t1_mat[2][3];
}

// Return nearest neighbor point value in a sphereical coordinate system
static void antennaGain( const float gain_data[STEPS][STEPS], const float polar_vec[3], float gain )
{
	int elevation, azimuth;
	
	elevation = nearestGapDeg( polar_vec[1], GAP );
	azimuth = nearestGapDeg( polar_vec[2], GAP );
	
	return gain_data[elevation][azimuth];
}

int signalStrength( const float 1a_pos_vec[3], const float 1v_pos_vec[3], const float 2a_pos_vec[3], 
				 	const float 2v_pos_vec[3], const int 1a_pos_vec[3], const int 1v_pos_vec[3],
				  	const int 2a_pos_vec[3], const int 2v_pos_vec[3], const float 1a_gain_data[STEPS][STEPS],
				  	const float 2a_gain_data[STEPS][STEPS], const int 1a_wavelength, const float 1a_t_power,
				  	const float 1b_r_sensitivity, float signal_strength ) 
{	
	float max_distance;
	
	max_distance =	(sqrt(t_gain * r_gain * 1a_t_power) * 1a_wavelength) / 
					(sqrt(LOSS_FACTOR * 1b_r_sensitivity) * 4 * pi());
	
	float 1a_mat, 1v_mat, 2a_mat, 2v_mat;
	
	
	return distance / max_distance;
}

void antennaRead( const string file_name, float out_gain[STEPS][STEPS], float t_wavelength, float t_power, float r_sens )
{
	TFileHandle file_handle;	// create a file handle variable 'myFileHandle'
	TFileIOResult io_result;	// create an IO result variable 'IOResult'
	int file_size;
	int t_wavelength;

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