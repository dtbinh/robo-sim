#ifndef antenna
#define antenna
#include "constants.h"

float signalStrength( const fvec3 a1_pos_vec, const fvec3 v1_pos_vec, const fvec3 a2_pos_vec,
				 	  const fvec3 v2_pos_vec, const ivec3 a1_rot_vec, const ivec3 v1_rot_vec,
				  	  const ivec3 a2_rot_vec, const ivec3 v2_rot_vec, const fvec3 a1_pol_vec,
					  const fvec3 a2_pol_vec, const fmatdata a1_gain_data, const fmatdata a2_gain_data,
					  const float a1_wavelength, const fvec3 a1_other, const fvec3 a2_other );

int isConnected( const fvec3 a1_pos_vec, const fvec3 v1_pos_vec, const fvec3 a2_pos_vec,
				 	  const fvec3 v2_pos_vec, const ivec3 a1_rot_vec, const ivec3 v1_rot_vec,
				  	  const ivec3 a2_rot_vec, const ivec3 v2_rot_vec, const fvec3 a1_pol_vec,
					  const fvec3 a2_pol_vec, const fmatdata a1_gain_data, const fmatdata a2_gain_data,
					  const float a1_wavelength, const fvec3 a1_other, const fvec3 a2_other );

void antennaRead( string file_name, float t_wavelength, float t_power, float r_sens,
			  	  float reflect_coef, fvec3 pol_vec, float axial_ratio,
    			  fmatdata out_gain );


#endif
