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

// USED FOR OFFLINE COMPILATION IN C (not robotC) 
// #include "Mymath.h"
// #include <math.h>
// #include <stdio.h>

#include "antenna_math.h"

// Multiply matrices. Technically this multiplies matrices which would be 4x4
// matrices with the last row always being [0 0 0 1]. Hardcoding this instead of
// inserting the row and doing a full 4x4 matrix multiply reduces computation
void matMatMult( const fmat34 a_mat, const fmat34 b_mat, fmat34 c_mat )
{
	c_mat[0][0] = a_mat[0][0] * b_mat[0][0] + a_mat[0][1] * b_mat[1][0] + a_mat[0][2] * b_mat[2][0];
	c_mat[0][1] = a_mat[0][0] * b_mat[0][1] + a_mat[0][1] * b_mat[1][1] + a_mat[0][2] * b_mat[2][1];
	c_mat[0][2] = a_mat[0][0] * b_mat[0][2] + a_mat[0][1] * b_mat[1][2] + a_mat[0][2] * b_mat[2][2];
	c_mat[0][3] = a_mat[0][0] * b_mat[0][3] + a_mat[0][1] * b_mat[1][3] + a_mat[0][2] * b_mat[2][3] + a_mat[0][3];
	c_mat[1][0] = a_mat[1][0] * b_mat[0][0] + a_mat[1][1] * b_mat[1][0] + a_mat[1][2] * b_mat[2][0];
	c_mat[1][1] = a_mat[1][0] * b_mat[0][1] + a_mat[1][1] * b_mat[1][1] + a_mat[1][2] * b_mat[2][1];
	c_mat[1][2] = a_mat[1][0] * b_mat[0][2] + a_mat[1][1] * b_mat[1][2] + a_mat[1][2] * b_mat[2][2];
	c_mat[1][3] = a_mat[1][0] * b_mat[0][3] + a_mat[1][1] * b_mat[1][3] + a_mat[1][2] * b_mat[2][3] + a_mat[1][3];
	c_mat[2][0] = a_mat[2][0] * b_mat[0][0] + a_mat[2][1] * b_mat[1][0] + a_mat[2][2] * b_mat[2][0];
	c_mat[2][1] = a_mat[2][0] * b_mat[0][1] + a_mat[2][1] * b_mat[1][1] + a_mat[2][2] * b_mat[2][1];
	c_mat[2][2] = a_mat[2][0] * b_mat[0][2] + a_mat[2][1] * b_mat[1][2] + a_mat[2][2] * b_mat[2][2];
	c_mat[2][3] = a_mat[2][0] * b_mat[0][3] + a_mat[2][1] * b_mat[1][3] + a_mat[2][2] * b_mat[2][3] + a_mat[2][3];
}

// Calculates the first three rows of the inverse of a 4x4 matrix, assuming
// the 4x4 matrix has bottom row of [0 0 0 1]
void matInverse( const fmat34 a_mat, fmat34 a_inv_mat )
{
	// Since the upper left 3x3 portion is an affine orthonormal rotation matrix,
	// inverse = the transpose
	a_inv_mat[0][0] = a_mat[0][0];
	a_inv_mat[0][1] = a_mat[1][0];
	a_inv_mat[0][2] = a_mat[2][0];
	a_inv_mat[1][0] = a_mat[0][1];
	a_inv_mat[1][1] = a_mat[1][1];
	a_inv_mat[1][2] = a_mat[2][1];
	a_inv_mat[2][0] = a_mat[0][2];
	a_inv_mat[2][1] = a_mat[1][2];
	a_inv_mat[2][2] = a_mat[2][2];
	// translation (rightmost) column is simply the upper left 3x3 -(a_inv_mat * translation vector)
	a_inv_mat[0][3] = -(a_inv_mat[0][0] * a_mat[0][3] + a_inv_mat[0][1] * a_mat[1][3] + a_inv_mat[0][2] * a_mat[2][3]);
	a_inv_mat[1][3] = -(a_inv_mat[1][0] * a_mat[0][3] + a_inv_mat[1][1] * a_mat[1][3] + a_inv_mat[1][2] * a_mat[2][3]);
	a_inv_mat[2][3] = -(a_inv_mat[2][0] * a_mat[0][3] + a_inv_mat[2][1] * a_mat[1][3] + a_inv_mat[2][2] * a_mat[2][3]);
	
}

// Calculate rotation matrix from Tait-Bryan angles given in integer degrees
//convert back to static type when done with debugging
void euler2Rot( const ivec3 rot_vec, fmat33 rot_mat )
{
	float c_heading, s_heading, c_elevation, s_elevation, c_bank, s_bank;
	
	c_heading = cosDegrees(rot_vec[0]);
	s_heading = sinDegrees(rot_vec[0]);	
	c_elevation = cosDegrees(rot_vec[1]);
	s_elevation = sinDegrees(rot_vec[1]);
	c_bank = cosDegrees(rot_vec[2]);
	s_bank = sinDegrees(rot_vec[2]);
	
	rot_mat[0][0] = c_elevation * c_heading;
	rot_mat[0][1] = -c_elevation * s_heading;
	rot_mat[0][2] = s_elevation;
	rot_mat[1][0] = c_bank * s_heading + c_heading * s_bank * s_elevation;
	rot_mat[1][1] = c_bank * c_heading - s_bank * s_elevation * s_heading;
	rot_mat[1][2] = -c_elevation * s_bank;
	rot_mat[2][0] = s_bank * s_heading - c_bank * c_heading * s_elevation;
	rot_mat[2][1] = c_heading * s_bank + c_bank * s_elevation * s_heading;
	rot_mat[2][2] = c_bank * c_elevation;
}

// Returns spherical coordinate direction [elevation, azimuth] in degrees
// based on cartesian input [X,Y,Z]
void cartToSpher( const fvec3 cart_vec, fvec3 polar_vec )
{
	// radius:
	polar_vec[0] = sqrt(cart_vec[0] * cart_vec[0] + cart_vec[1] * cart_vec[1] + cart_vec[2] * cart_vec[2]);
	
	// theta: (elevation)
	polar_vec[1] = acosDegrees(cart_vec[2] / polar_vec[0]);
	if (polar_vec[1] < 0) polar_vec[1] = 360 + polar_vec[1];
	
	// phi: (azimuth)
	polar_vec[2] = atanDegrees(cart_vec[1] / cart_vec[0]);
	if (polar_vec[2] < 0) polar_vec[2] = 360 + polar_vec[2];
}

// Returns the nearest integer which fits within the multiple of gap
int nearestGapDeg( float degrees, int gap )
{
	return (int)(degrees / (float)gap + 0.5);// * gap;
}

void buildMat( const fvec3 pos_vec, const ivec3 rot_vec, fmat34 mat )
{
	float rot_mat[3][3];
	euler2Rot(rot_vec, rot_mat);
	
	mat[0][0] = rot_mat[0][0];
	mat[0][1] = rot_mat[0][1];
	mat[0][2] = rot_mat[0][2];
	mat[0][3] = pos_vec[0];
	mat[1][0] = rot_mat[1][0];
	mat[1][1] = rot_mat[1][1];
	mat[1][2] = rot_mat[1][2];
	mat[1][3] = pos_vec[1];
	mat[2][0] = rot_mat[2][0];
	mat[2][1] = rot_mat[2][1];
	mat[2][2] = rot_mat[2][2];
	mat[2][3] = pos_vec[2];
}
