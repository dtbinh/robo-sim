#include <math.h>
#include "Mymath.h"
#include <stdio.h>

	//cos
	float cosDegrees(int degrees)
	{  
		return (float)cos((float)degrees * pi()/180.f);
	}
	//sin
	float sinDegrees(int degrees)
	{
		return sin((float)degrees * pi()/180.f);
	}
	//tan
	float tanDegrees(int degrees)
	{
		return sinDegrees(degrees) / cosDegrees(degrees);
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
