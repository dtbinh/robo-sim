#ifndef antenna_math
#define antenna_math
#include "constants.h"

void matMatMult( const fmat34 a_mat, const fmat34 b_mat, fmat34 c_mat );
void matInverse( const fmat34 a_mat, fmat34 a_inv_mat );
void cartToSpher( const fvec3 cart_vec, fvec3 polar_vec );
int nearestGapDeg( float degrees, int gap );
void buildMat( const fvec3 pos_vec, const ivec3 rot_vec, fmat34 mat );
//convert back to static type when done with debugging
void euler2Rot( const ivec3 rot_vec, fmat33 rot_mat );
static float PLF( const fvec3 a1_pol_in_a2_vec, const float a1_axial_ratio,
		   const fvec3 a2_pol_vec, const float a2_axial_ratio );

#endif
