// -------------------------------------------------------------------
// Conventions:
// 
// Coordinate System:
// Right handed coordinate system
// Forward is positive X axis
// Left (Port) is positive Y axis
// Rigt (Starboard) is negative Y axis
// Up is positive Z axis
//
// Rotation Matrix:
// Heading		->	rotation about Z, positive X axis is origin
// Elevation	->	rotation about Y, positive Z axis is origin
// Bank			->	rotation about X, positive Y axis is origin
// Order:	object Heading, object Elevation, object Bank (Tait-Bryan)
// 			Z, Y', X''
//			This causes the rotation matrix construction to be R = XYZ		
//
// We choose this order in order to not add to the Heading angle in world
// coordinates during the latter two rotations. Thus we can use world
// oriented sensors (compass) to measure object Heading, since world
// Heading = object Heading, and use it for our rotation matrix construction.
//
// The rotation position matrix without camera or scaling is formed as:
// [R00 R01 R02 Tx]
// [R10 R11 R12 Ty]
// [R20 R21 R22 Tz]
// [0   0   0   1 ]
// 
// However, since the bottom row is always constant, we store and
// calculate assuming those values, so it becomes:
// [R00 R01 R02 Tx]
// [R10 R11 R12 Ty]
// [R20 R21 R22 Tz]
// with various mathematical operations (multiplication, etc.) hard-coded
// -------------------------------------------------------------------

void 3x3MatMatMult( const float A[3][3], const float B[3][3], float C[3][3] )
{
	C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
	C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
	C[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
	C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
	C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
	C[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
	C[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
	C[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
	C[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2];
}

// Multiply matrices. Technically this multiplies matrices which would be 4x4
// matrices with the last row always being [0 0 0 1]. Hardcoding this instead of
// inserting the row and doing a full 4x4 matrix multiply reduces computation
void MatMatMult( const float A[3][4], const float B[3][4], float C[3][4] )
{
	C[0][0] = A[0][0] * B[0][0] + A[0][1] * B[1][0] + A[0][2] * B[2][0];
	C[0][1] = A[0][0] * B[0][1] + A[0][1] * B[1][1] + A[0][2] * B[2][1];
	C[0][2] = A[0][0] * B[0][2] + A[0][1] * B[1][2] + A[0][2] * B[2][2];
	C[0][3] = A[0][0] * B[0][3] + A[0][1] * B[1][3] + A[0][2] * B[2][3] + A[0][3];
	C[1][0] = A[1][0] * B[0][0] + A[1][1] * B[1][0] + A[1][2] * B[2][0];
	C[1][1] = A[1][0] * B[0][1] + A[1][1] * B[1][1] + A[1][2] * B[2][1];
	C[1][2] = A[1][0] * B[0][2] + A[1][1] * B[1][2] + A[1][2] * B[2][2];
	C[1][3] = A[1][0] * B[0][3] + A[1][1] * B[1][3] + A[1][2] * B[2][3] + A[1][3];
	C[2][0] = A[2][0] * B[0][0] + A[2][1] * B[1][0] + A[2][2] * B[2][0];
	C[2][1] = A[2][0] * B[0][1] + A[2][1] * B[1][1] + A[2][2] * B[2][1];
	C[2][2] = A[2][0] * B[0][2] + A[2][1] * B[1][2] + A[2][2] * B[2][2];
	C[2][3] = A[2][0] * B[0][3] + A[2][1] * B[1][3] + A[2][2] * B[2][3] + A[2][3];
}

void MatVecMult( const float A[3][3], const float x[3] , float b[3] )
{
	b[0] = A[0][0] * b[0] + A[0][1] * b[1] + A[0][2] * b[2];
	b[1] = A[1][0] * b[0] + A[1][1] * b[1] + A[1][2] * b[2];
	b[2] = A[2][0] * b[0] + A[2][1] * b[1] + A[2][2] * b[2];
}

void MatTranspose( const float A[3][3], float A_t[3][3] )
{

	A_t[0][0] = A[0][0];
	A_t[0][1] = A[1][0];
	A_t[0][2] = A[2][0];
	A_t[1][0] = A[0][1];
	A_t[1][1] = A[1][1];
	A_t[1][2] = A[2][1];
	A_t[2][0] = A[0][2];
	A_t[2][1] = A[1][2];
	A_t[2][2] = A[2][2];
	
}

// Calculates the first three rows of the inverse of a 4x4 matrix, assuming
// the 4x4 matrix has bottom row of [0 0 0 1]
void MatInverse( const float A[3][4], float A_i[3][4] )
{
	// Since the upper left 3x3 portion is an affine orthonormal rotation matrix,
	// inverse = the transpose
	A_i[0][0] = A[0][0];
	A_i[0][1] = A[1][0];
	A_i[0][2] = A[2][0];
	A_i[1][0] = A[0][1];
	A_i[1][1] = A[1][1];
	A_i[1][2] = A[2][1];
	A_i[2][0] = A[0][2];
	A_i[2][1] = A[1][2];
	A_i[2][2] = A[2][2];
	// translation (rightmost) column is simply -A_i * translation vector
	A_i[0][3] = A_i[0][0] * A_i[0][3] + A_i[0][1] * A_i[1][3] + A_i[0][2] * A_i[2][3];
	A_i[1][3] = A_i[1][0] * A_i[0][3] + A_i[1][1] * A_i[1][3] + A_i[1][2] * A_i[2][3];
	A_i[2][3] = A_i[2][0] * A_i[0][3] + A_i[2][1] * A_i[1][3] + A_i[2][2] * A_i[2][3];
	
}

// Calculate rotation matrix from Tait-Bryan angles given in integer degrees
static void Euler2RotMat( const int heading, const int elevation, const int bank, float RotMat[3][3] )
{
	float c_heading, s_heading, c_elevation, s_elevation, c_bank, s_bank;
	
	c_heading = cosDegrees(heading);
	s_heading = sinDegrees(heading);	
	c_elevation = cosDegrees(elevation);
	s_elevation = sinDegrees(elevation);
	c_bank = cosDegrees(bank);
	s_bank = sinDegrees(bank);
	
	RotMat[0][0] = c_elevation * c_heading;
	RotMat[0][1] = -c_elevation * s_heading;
	RotMat[0][2] = s_elevation;
	RotMat[1][0] = c_bank * s_heading + c_heading * s_bank * s_elevation;
	RotMat[1][1] = c_bank * c_heading - s_bank * s_elevation * s_heading;
	RotMat[1][2] = -c_elevation * s_bank;
	RotMat[2][0] = s_bank * s_heading - c_bank * c_heading * s_elevation;
	RotMat[2][1] = c_heading * s_bank + c_bank * s_elevation * s_heading;
	RotMat[2][2] = c_bank * c_elevation;
}

// Returns spherical coordinate direction [elevation, azimuth] in degrees
// based on cartesian input [X,Y,Z]
void Cart2Polar( const float cart[3], float polar[2] )
{
	// radius:
	float radius;
	radius = sqrt(cart[0] * cart[0] + cart[1] * cart[1] + cart[2] * cart[2]);
	// not needed for direction only
	
	// theta: (elevation)
	polar[1] = acosDegrees(cart[2] / polar[0]);
	
	// phi: (azimuth)
	polar[2] = atanDegrees(cart[1] / cart[0]);
}

int round_gap( float f )
{
	int g;
	const int gap = 10;
	
	g = f / gap;
	if(f>0) return (int)(f + 0.5) * gap;
	else    return (int)(f - 0.5) * gap;
}

void BuildMat( const int angle[3], const float pos[3], float Mat[3][4] )
{
	float Rot[3][3];
	Euler2RotMat(angle[0], angle[1], angle[2], Rot);
	
	Mat[0][0] = Rot[0][0];
	Mat[0][1] = Rot[0][1];
	Mat[0][2] = Rot[0][2];
	Mat[0][3] = pos[0];
	Mat[1][0] = Rot[1][0];
	Mat[1][1] = Rot[1][1];
	Mat[1][2] = Rot[1][2];
	Mat[1][3] = pos[1];
	Mat[2][0] = Rot[2][0];
	Mat[2][1] = Rot[2][1];
	Mat[2][2] = Rot[2][2];
	Mat[2][3] = pos[2];
}