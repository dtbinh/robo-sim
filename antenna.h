#ifndef antenna
#define antenna
#include "constants.h"

float signalStrength( const fvec3 a1_pos_vec, const fvec3 v1_pos_vec, const fvec3 a2_pos_vec, 
				 	  const fvec3 v2_pos_vec, const ivec3 a1_rot_vec, const ivec3 v1_rot_vec,
				  	  const ivec3 a2_rot_vec, const ivec3 v2_rot_vec, const fvec3 a1_pol_vec,
					  const float a1_axial_ratio, const fvec3 a2_pol_vec, const float a2_axial_ratio,
					  const fmatdata a1_gain_data, const fmatdata a2_gain_data,
					  const float a1_wavelength, const float a1_t_power, const float a2_r_sensitivity,
					  const float a1_reflect_coef, const float a2_reflect_coef );

int isConnected( const fvec3 a1_pos_vec, const fvec3 v1_pos_vec, const fvec3 a2_pos_vec, 
			   	 const fvec3 v2_pos_vec, const ivec3 a1_rot_vec, const ivec3 v1_rot_vec,
				 const ivec3 a2_rot_vec, const ivec3 v2_rot_vec, const fvec3 a1_pol_vec,
				 const float a1_axial_ratio, const fvec3 a2_pol_vec, const float a2_axial_ratio,
				 const fmatdata a1_gain_data, const fmatdata a2_gain_data,
				 const float a1_wavelength, const float a1_t_power, const float a2_r_sensitivity,
				 const float a1_reflect_coef, const float a2_reflect_coef );
				
void antennaRead( const char file_name[], float t_wavelength, float t_power, float r_sens,
			  	  float reflect_coef, fvec3 pol_vec, float axial_ratio,
    			  fmatdata out_gain );

#endif