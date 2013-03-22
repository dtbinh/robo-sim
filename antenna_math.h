#ifndef antenna_math
#define antenna_math

void matMatMult( const float a_mat[3][4], const float b_mat[3][4], float c_mat[3][4] );
void matInverse( const float a_mat[3][4], float a_inv_mat[3][4] );
void cartToSpher( const float cart_vec[3], float polar_vec[3] );
int nearestGapDeg( float degrees, int gap );
void buildMat( const float pos_vec[3], const int rot_vec[3], float mat[3][4] );
//convert back to static type when done with debugging
void euler2Rot( const int rot_vec[3], float rot_mat[3][3] );

#endif