#ifndef antenna
#define antenna
#include "constants.h"

float signalStrength( const float *a1_pos_vec, const float *v1_pos_vec, const float *a2_pos_vec, 
				 	  const float *v2_pos_vec, const int *a1_rot_vec, const int *v1_rot_vec,
				  	  const int *a2_rot_vec, const int *v2_rot_vec, const float *a1_pol_vec,
					  const float a1_axial_ratio, const float *a2_pol_vec, const float a2_axial_ratio,
					  const float *a1_gain_data, const float *a2_gain_data,
					  const float a1_wavelength, const float a1_t_power, const float a2_r_sensitivity,
					  const float a1_reflect_coef, const float a2_reflect_coef );

int isConnected( const float *a1_pos_vec, const float *v1_pos_vec, const float *a2_pos_vec, 
			   	 const float *v2_pos_vec, const int *a1_rot_vec, const int *v1_rot_vec,
				 const int *a2_rot_vec, const int *v2_rot_vec, const float *a1_pol_vec,
				 const float a1_axial_ratio, const float *a2_pol_vec, const float a2_axial_ratio,
				 const float *a1_gain_data, const float *a2_gain_data,
				 const float a1_wavelength, const float a1_t_power, const float a2_r_sensitivity,
				 const float a1_reflect_coef, const float a2_reflect_coef );
				
void antennaRead( const char *file_name, float t_wavelength, float t_power, float r_sens,
			  	  float reflect_coef, float *pol_vec, float axial_ratio,
    			  float *out_gain );

#endif