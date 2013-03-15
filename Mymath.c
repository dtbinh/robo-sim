#include <math.h>
#define PI 3.141592654
  
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
	// END OF SECTION OF CODE // 
