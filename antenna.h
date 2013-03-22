#ifndef antenna
#define antenna
#include "constants.h"

float signalStrength( const float a1_pos_vec[3], const float v1_pos_vec[3], const float a2_pos_vec[3], 
				 	  const float v2_pos_vec[3], const int a1_rot_vec[3], const int v1_rot_vec[3],
				  	  const int a2_rot_vec[3], const int v2_rot_vec[3], const float a1_gain_data[STEPS][STEPS],
				  	  const float a2_gain_data[STEPS][STEPS], const int a1_wavelength, const float a1_t_power,
 					  const float a2_r_sensitivity );

int isConnected( const float a1_pos_vec[3], const float v1_pos_vec[3], const float a2_pos_vec[3], 
				 const float v2_pos_vec[3], const int a1_rot_vec[3], const int v1_rot_vec[3],
				 const int a2_rot_vec[3], const int v2_rot_vec[3], const float a1_gain_data[STEPS][STEPS],
				 const float a2_gain_data[STEPS][STEPS], const int a1_wavelength, const float a1_t_power,
				 const float a2_r_sensitivity );
				
void antennaRead( const char file_name[], float out_gain[STEPS][STEPS], float *t_wavelength, float *t_power, float *r_sens );			

#endif