#ifndef antenna_math
#define antenna_math

void matMatMult( const float *a_mat, const float *b_mat, float *c_mat );
void matInverse( const float *a_mat, float *a_inv_mat );
void cartToSpher( const float *cart_vec, float *polar_vec );
int nearestGapDeg( float degrees, int gap );
void buildMat( const float *pos_vec, const int *rot_vec, float *mat );
//convert back to static type when done with debugging
void euler2Rot( const int *rot_vec, float *rot_mat );

#endif