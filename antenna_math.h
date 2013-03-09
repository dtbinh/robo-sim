void 3x3MatMatMult( const float A[3][3], const float B[3][3], float C[3][3] );
void MatMatMult( const float A[3][4], const float B[3][4], float C[3][4] );
void MatVecMult( const float A[3][3], const float x[3] , float b[3] );
void MatTranspose( const float A[3][3], float A_t[3][3] );
void MatInverse( const float A[3][4] float A_i[3][4] );
void Euler2RotMat( const int heading, const int elevation, const int bank, float RotMat[3][3] );
void Cart2Polar( const float cart[3], float polar[2] );
int round_gap( float f );
void BuildMat( const int angle[3], const float pos[3], float Mat[3][4] )