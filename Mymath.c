#include <math.h>
#include "Mymath.h"
 
	//cos
	float cosDegrees(float degrees)
	{  
		return cos(degrees*PI/180);
	}
	//sin
	float sinDegrees(float degrees)
	{
		return sin(degrees*PI/180);
	}
	//tan
	float tanDegrees(float degrees)
	{
		return sinDegrees(degrees)/cosDegrees(degrees);
	}	
	//arccos
	float acosDegrees(float value)
	{
		return (180/PI)*acos(value);
	}
	//arcsin
	float asinDegrees(float value)
	{
		return (180/PI)*asin(value);
	}
	//arctan
	float atanDegrees(float value)
	{
		return (180/PI)*atan(value);
	}
	
	float pi(void)
	{
		return PI;
	}
	// END OF SECTION OF CODE // 
