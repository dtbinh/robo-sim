static void relativeVectors( const float 1a_mat[3][4], const float 1v_mat[3][4], const float 2a_mat[3][4],
	const float 2v_mat[3][4], float 1a_from_2a[3], float 2a_from_1a[3]);
	
static void antennaGain( const float gain_data[STEPS][STEPS], const float polar_vec[3], float gain );

float signalStrength( const float 1a_pos_vec[3], const float 1v_pos_vec[3], const float 2a_pos_vec[3], 
				 	  const float 2v_pos_vec[3], const int 1a_rot_vec[3], const int 1v_rot_vec[3],
				  	  const int 2a_rot_vec[3], const int 2v_rot_vec[3], const float 1a_gain_data[STEPS][STEPS],
				  	  const float 2a_gain_data[STEPS][STEPS], const int 1a_wavelength, const float 1a_t_power,
 					  const float 2a_r_sensitivity );

int isConnected( const float 1a_pos_vec[3], const float 1v_pos_vec[3], const float 2a_pos_vec[3], 
				 const float 2v_pos_vec[3], const int 1a_rot_vec[3], const int 1v_rot_vec[3],
				 const int 2a_rot_vec[3], const int 2v_rot_vec[3], const float 1a_gain_data[STEPS][STEPS],
				 const float 2a_gain_data[STEPS][STEPS], const int 1a_wavelength, const float 1a_t_power,
				const float 2a_r_sensitivity );
				
void antennaRead( const string file_name, float out_gain[STEPS][STEPS], float t_wavelength, float t_power, float r_sens );			