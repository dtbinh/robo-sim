#include "antenna_math.h"

// Calculate two vectors:
// ant2_from1: Vector [x y z] in ant_1 coordinates that points directly at ant_2
// ant1_from2: Vector [x y z] in ant_2 coordinates that points directly at ant_1
static void AntVectors( const float ant_1[3][4], const float robot_1[3][4], const float ant_2[3][4],
						const float robot_2[3][4], float ant2_from1[3], float ant1_from2[3])
{
	float[3][4] t1_Mat, t2_Mat, t3_Mat;
	
	// calculating i_ant_1 * i_robot_1 * robot_2 * ant_2 yields ant_2 from the
	// coordinate system of ant_1
	
	MatInverse(ant_1, t1_Mat);
	MatInverse(robot_1, t2_Mat);
	
	MatMatMult(t1_Mat, t2_Mat, t3_Mat);
	MatMatMult(t3_Mat, robot_2, t1_Mat);
	MatMatMult(t1_Mat, ant_2, t3_Mat);		// t3_Mat is now ant_2 from ant_1
	
	ant2_from1[0] = t3_Mat[0][3];
	ant2_from1[1] = t3_Mat[1][3];
	ant2_from1[2] = t3_Mat[2][3];
	
	MatInverse(t3_Mat, t1_Mat);				// inverting to get ant_1 from ant_2's perspective
	
	ant1_from2[0] = t1_Mat[0][3];
	ant1_from2[1] = t1_Mat[1][3];
	ant1_from2[2] = t1_Mat[2][3];
}

// Return nearest neighbor point value in a sphereical coordinate system
static void AntGain( const float ant_gain[36][36], const float direction[2], float gain )
{
	int elevation, azimuth;
	
	elevation = quantize_deg( direction[0] );
	azimuth = quantize_deg( direction[1] );
	
	return ant_gain[elevation][azimuth];
}

int is_connected( const float t_wavelenth, const float t_power, const float t_gain[36][36], const float t_pos[3],
				  const float t_angle[3], const float r_sens, const float r_gain[36][36], const float r_pos[3],
				  const float r_angle[3] )
{
	float distance = 
	
	const float wavelength = 0.12491;	// 2.4 GHz in meters
	const float loss_factor = 1; 		// Increase for added loss through atmosphere
	float max_distance;
	
	max_distance =	(sqrt(t_gain * r_gain * t_power) * wavelength) / 
					(sqrt(loss_factor * r_sens) * 4 * pi());
	
	return distance < max_distance;
}

void AntRead( const string myFileName, float outGain[36][36], float t_wavelength, float t_power, float r_sens )
{
	TFileHandle myFileHandle;	// create a file handle variable 'myFileHandle'
	TFileIOResult myIOResult;	// create an IO result variable 'IOResult'
	int myFileSize;
	int t_wavelength;

	OpenRead(myFileHandle, IOResult, myFileName, myFileSize);  // open for read: "myFile.txt",
	                                                           // storing its size in 'myFileSize'
	ReadFloat(myFileHandle, IOResult, t_wavelength);
	ReadFloat(myFileHandle, IOResult, t_power);
	ReadFloat(myFileHandle, IOResult, r_sens);
	for(int i = 0; i < 36; i++)
	{
		for(int j = 0; j < 36; j++)
		{
			ReadFloat(myFileHandle, IOResult, outGain[i][j]);
		}
	}
	
}