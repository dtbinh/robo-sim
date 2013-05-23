//#pragma config(Sensor, S1,     sonarSensor1,         sensorSONAR)
//#pragma config(Sensor, S2,     sonarSensor2,         sensorSONAR)

#include "XbeeTools.h"
#include "C:\Program Files (x86)\Robomatter Inc\ROBOTC Development Environment\Sample Programs\NXT\3rd Party Sensor Drivers\drivers\firgelli-linearact.h"

//#include "antenna_math.h"
//#include "constants.h"
//#include "antenna.h"

float exp_time;
int direction_actuator;
int step=36, gap=10;
//float *mat,
typedef const float myarray[3][3];
typedef const float myarray1[3][4];

void matInverse(myarray a_mat, myarray1 a_inv_mat)
{
////////////////////////////////

a_inv_mat[0][0]=a_mat[0][0];
a_inv_mat[0][1]=a_mat[0][1];
a_inv_mat[0][2]=a_mat[0][2];
a_inv_mat[1][0]=a_mat[1][0];
a_inv_mat[1][1]=a_mat[1][1];
a_inv_mat[1][2]=a_mat[1][2];
a_inv_mat[2][0]=a_mat[2][0];
a_inv_mat[2][1]=a_mat[2][1];
a_inv_mat[2][2]=a_mat[2][2];

}

	//// translation (rightmost) column is simply the upper left 3x3 -(a_inv_mat * translation vector)
	//*(a_inv_mat+0)+3 = -(*(a_inv_mat+0)+0)) * (*(a_mat+0)+3);// + a_inv_mat[0][1] * a_mat[1][3] + a_inv_mat[0][2] * a_mat[2][3]);
//	a_inv_mat[1][3] = -(a_inv_mat[1][0] * a_mat[0][3] + a_inv_mat[1][1] * a_mat[1][3] + a_inv_mat[1][2] * a_mat[2][3]);
//	a_inv_mat[2][3] = -(a_inv_mat[2][0] * a_mat[0][3] + a_inv_mat[2][1] * a_mat[1][3] + a_inv_mat[2][2] * a_mat[2][3]);

//}

//void matMatMult( const float *a_mat, const float *b_mat, float *c_mat )
//{
//	c_mat[0][0] = a_mat[0][0] * b_mat[0][0] + a_mat[0][1] * b_mat[1][0] + a_mat[0][2] * b_mat[2][0];
//	c_mat[0][1] = a_mat[0][0] * b_mat[0][1] + a_mat[0][1] * b_mat[1][1] + a_mat[0][2] * b_mat[2][1];
//	c_mat[0][2] = a_mat[0][0] * b_mat[0][2] + a_mat[0][1] * b_mat[1][2] + a_mat[0][2] * b_mat[2][2];
//	c_mat[0][3] = a_mat[0][0] * b_mat[0][3] + a_mat[0][1] * b_mat[1][3] + a_mat[0][2] * b_mat[2][3] + a_mat[0][3];
//	c_mat[1][0] = a_mat[1][0] * b_mat[0][0] + a_mat[1][1] * b_mat[1][0] + a_mat[1][2] * b_mat[2][0];
//	c_mat[1][1] = a_mat[1][0] * b_mat[0][1] + a_mat[1][1] * b_mat[1][1] + a_mat[1][2] * b_mat[2][1];
//	c_mat[1][2] = a_mat[1][0] * b_mat[0][2] + a_mat[1][1] * b_mat[1][2] + a_mat[1][2] * b_mat[2][2];
//	c_mat[1][3] = a_mat[1][0] * b_mat[0][3] + a_mat[1][1] * b_mat[1][3] + a_mat[1][2] * b_mat[2][3] + a_mat[1][3];
//	c_mat[2][0] = a_mat[2][0] * b_mat[0][0] + a_mat[2][1] * b_mat[1][0] + a_mat[2][2] * b_mat[2][0];
//	c_mat[2][1] = a_mat[2][0] * b_mat[0][1] + a_mat[2][1] * b_mat[1][1] + a_mat[2][2] * b_mat[2][1];
//	c_mat[2][2] = a_mat[2][0] * b_mat[0][2] + a_mat[2][1] * b_mat[1][2] + a_mat[2][2] * b_mat[2][2];
//	c_mat[2][3] = a_mat[2][0] * b_mat[0][3] + a_mat[2][1] * b_mat[1][3] + a_mat[2][2] * b_mat[2][3] + a_mat[2][3];
//}

//// Calculates the first three rows of the inverse of a 4x4 matrix, assuming
//// the 4x4 matrix has bottom row of [0 0 0 1]

//// Calculate rotation matrix from Tait-Bryan angles given in integer degrees
////convert back to static type when done with debugging
//void euler2Rot (int rot_vec[3], float rot_mat[3][3] )
//{
//	float c_heading, s_heading, c_elevation, s_elevation, c_bank, s_bank;

//	c_heading = cosDegrees(rot_vec[0]);
//	s_heading = sinDegrees(rot_vec[0]);
//	c_elevation = cosDegrees(rot_vec[1]);
//	s_elevation = sinDegrees(rot_vec[1]);
//	c_bank = cosDegrees(rot_vec[2]);
//	s_bank = sinDegrees(rot_vec[2]);

//	rot_mat[0][0] = c_elevation * c_heading;
//	rot_mat[0][1] = -c_elevation * s_heading;
//	rot_mat[0][2] = s_elevation;
//	rot_mat[1][0] = c_bank * s_heading + c_heading * s_bank * s_elevation;
//	rot_mat[1][1] = c_bank * c_heading - s_bank * s_elevation * s_heading;
//	rot_mat[1][2] = -c_elevation * s_bank;
//	rot_mat[2][0] = s_bank * s_heading - c_bank * c_heading * s_elevation;
//	rot_mat[2][1] = c_heading * s_bank + c_bank * s_elevation * s_heading;
//	rot_mat[2][2] = c_bank * c_elevation;
//}

//// Returns spherical coordinate direction [elevation, azimuth] in degrees
//// based on cartesian input [X,Y,Z]
//void cartToSpher( const float *cart_vec, float *polar_vec )
//{
//	// radius:
//	polar_vec[0] = sqrt(cart_vec[0] * cart_vec[0] + cart_vec[1] * cart_vec[1] + cart_vec[2] * cart_vec[2]);

//	// theta: (elevation)
//	polar_vec[1] = acos(cart_vec[2] / polar_vec[0])*180/PI;
//	if (polar_vec[1] < 0) polar_vec[1] = 360 + polar_vec[1];

//	// phi: (azimuth)
//	polar_vec[2] = atan(cart_vec[1] / cart_vec[0])*180/PI;
//	if (polar_vec[2] < 0) polar_vec[2] = 360 + polar_vec[2];
//}

//// Returns the nearest integer which fits within the multiple of gap
//int nearestGapDeg( float degrees, int gap )
//{
//	return (int)(degrees / (float)gap + 0.5);// * gap;
//}

//void buildMat( const float pos_vec[3], const int rot_vec[3], float mat[3][4] )
//{
//	float rot_mat[3][3];
//	euler2Rot(rot_vec, rot_mat);

//	mat[0][0] = rot_mat[0][0];
//	mat[0][1] = rot_mat[0][1];
//	mat[0][2] = rot_mat[0][2];
//	mat[0][3] = pos_vec[0];
//	mat[1][0] = rot_mat[1][0];
//	mat[1][1] = rot_mat[1][1];
//	mat[1][2] = rot_mat[1][2];
//	mat[1][3] = pos_vec[1];
//	mat[2][0] = rot_mat[2][0];
//	mat[2][1] = rot_mat[2][1];
//	mat[2][2] = rot_mat[2][2];
//	mat[2][3] = pos_vec[2];
//}


///////////////////////////////////////////////


/////////////////////////////
//static void relativeVectors( const float *a1_mat, const float *v1_mat, const float *a2_mat,
//							 const float *v2_mat, float *a1_from_a2, float *a2_from_a1,
//							 const float *a1_pol, float *a1_pol_in_a2_vec)
//{
//	float t1_mat[3][4];
//	float t2_mat[3][4];
//	float t3_mat[3][4];

//	// calculating i_ant_1 * i_robot_1 * robot_2 * ant_2 yields ant_2 from the
//	// coordinate system of ant_1

//	matInverse(a1_mat, t1_mat);
//	matInverse(v1_mat, t2_mat);

//	matMatMult(t1_mat, t2_mat, t3_mat);
//	matMatMult(t3_mat, v2_mat, t1_mat);
//	matMatMult(t1_mat, a2_mat, t3_mat);		// t3_Mat is now ant_2 from ant_1

//	a2_from_a1[0] = t3_mat[0][3];
//	a2_from_a1[1] = t3_mat[1][3];
//	a2_from_a1[2] = t3_mat[2][3];

//	matInverse(t3_mat, t1_mat);				// inverting to get ant_1 from ant_2's perspective

//	a1_from_a2[0] = t1_mat[0][3];
//	a1_from_a2[1] = t1_mat[1][3];
//	a1_from_a2[2] = t1_mat[2][3];

//	// calculate antenna 1 polarization vector in antenna 3's coordinate system
//	a1_pol_in_a2_vec[0] = t3_mat[0][0] * a1_pol[0] + t3_mat[0][1] * a1_pol[1] + t3_mat[0][2] * a1_pol[2];
//	a1_pol_in_a2_vec[1] = t3_mat[1][0] * a1_pol[0] + t3_mat[1][1] * a1_pol[1] + t3_mat[1][2] * a1_pol[2];
//	a1_pol_in_a2_vec[2] = t3_mat[2][0] * a1_pol[0] + t3_mat[2][1] * a1_pol[1] + t3_mat[2][2] * a1_pol[2];

//}

//// Return nearest neighbor point value in a sphereical coordinate system
//static float antennaGain( const float *gain_data, const float *polar_vec )
//{
//	int elevation, azimuth;

//	elevation = nearestGapDeg( polar_vec[1], GAP );
//	azimuth = nearestGapDeg( polar_vec[2], GAP );

////	return gain_data[elevation][azimuth];
//}

////	returns signal strength, where 0 = perfect strength (zero distance)
//float signalStrength(const float *a1_pos_vec, const float *v1_pos_vec, const float *a2_pos_vec,
//				 	  const float *v2_pos_vec, const int *a1_rot_vec, const int *v1_rot_vec,
//				  	  const int *a2_rot_vec, const int *v2_rot_vec, const float *a1_pol_vec,
//					  const float a1_axial_ratio, const float *a2_pol_vec, const float a2_axial_ratio,
//					  const float a1_gain_data[step][step], const float a2_gain_data[step][step],
//					  const float a1_wavelength, const float a1_t_power, const float a2_r_sensitivity,
//					  const float a1_reflect_coef, const float a2_reflect_coef )
//{
//	// build matrix representations
//	float a1_mat[3][4], v1_mat[3][4], a2_mat[3][4], v2_mat[3][4];
//	buildMat(a1_pos_vec, a1_rot_vec, a1_mat);
//	buildMat(v1_pos_vec, v1_rot_vec, v1_mat);
//	buildMat(a2_pos_vec, a2_rot_vec, a2_mat);
//	buildMat(v2_pos_vec, v2_rot_vec, v2_mat);

//	// calculate relative vector directions
//	float a1_from_a2_vec[3], a2_from_a1_vec[3];
//	relativeVectors(a1_mat, v1_mat, a2_mat, v2_mat, a1_from_a2_vec, a2_from_a1_vec, a1_pol_vec, a1_pol_in_a2_vec);

//	// convert to polar coordinates
//	float a1_from_a2_pol_vec[3], a2_from_a1_pol_vec[3];
//	cartToSpher(a1_from_a2_vec, a1_from_a2_pol_vec);
//	cartToSpher(a2_from_a1_vec, a2_from_a1_pol_vec);

//	// find antenna gain
//	float t_gain, r_gain;
//	t_gain = antennaGain(a1_gain_data, a2_from_a1_pol_vec);
//	r_gain = antennaGain(a2_gain_data, a1_from_a2_pol_vec);

//	// find polarization loss
//	float pol_loss;
//	pol_loss = PLF(a1_pol_in_a2_vec, a1_axial_ratio, a2_pol_vec, a2_axial_ratio);

//	// calculate max distance where signal would be received
//	float max_distance;
//	max_distance =	(sqrt(t_gain * r_gain * a1_t_power) * a1_wavelength) / (sqrt(LOSS_FACTOR * a2_r_sensitivity) * 4 *PI);


//	float distance, power_received;
//	distance = sqrt(a1_from_a2_vec[0] * a1_from_a2_vec[0] + a1_from_a2_vec[1] * a1_from_a2_vec[1] +
//					a1_from_a2_vec[2] * a1_from_a2_vec[2]);
//	power_received = a1_t_power * t_gain * r_gain * (1 - a1_reflect_coef) * (1 - a1_reflect_coef) * pol_loss *
//					 (a1_wavelength / (4 * PI * distance));

//	// first coordinate of polar vector is the distance.
//	// 0 stregth: distance = max distance. 1 strength: distance = 0
//	return power_received / a2_r_sensitivity;
//}

////	returns signal strength, where 0 = extent of reception, and 1 = zero distance
//int isConnected( const float *a1_pos_vec, const float *v1_pos_vec, const float *a2_pos_vec,
//				 const float *v2_pos_vec, const int *a1_rot_vec, const int *v1_rot_vec,
//				 const int *a2_rot_vec, const int *v2_rot_vec, const float *a1_pol_vec,
//				 const float a1_axial_ratio, const float *a2_pol_vec, const float a2_axial_ratio,
//				 const float *a1_gain_data, const float *a2_gain_data,
//				 const float a1_wavelength, const float a1_t_power, const float a2_r_sensitivity,
//				 const float a1_reflect_coef, const float a2_reflect_coef )
//{
//	float signal_strength;
//	signal_strength = signalStrength(a1_pos_vec, v1_pos_vec, a2_pos_vec, v2_pos_vec, a1_rot_vec[3],
//									 v1_rot_vec, a2_rot_vec, v2_rot_vec, a1_pol_vec, a1_axial_ratio,
//									 a2_pol_vec, a2_axial_ratio, a1_gain_data, a2_gain_data,
//					 				 a1_wavelength, a1_t_power, a2_r_sensitivity, a1_reflect_coef,
//									 a2_reflect_coef);

//	return signal_strength > 1;
//}
///*
//void antennaRead( const char file_name[], float out_gain[STEPS][STEPS], float pol_vec[3],
//				  float *axial_ratio, float *t_wavelength, float *t_power, float *r_sens, float *reflect_coef )
//{
//	FILE *f;
//	int i;
//	int j;
//	float temp;

//	if (f = fopen(file_name, "rb"))
//	{
//		fread(&temp, 4, 1, f);
//		*t_wavelength = temp;

//		fread(&temp, 4, 1, f);
//		*t_power = temp;

//		fread(&temp, 4, 1, f);
//		*r_sens = temp;

//		fread(&temp, 4, 1, f);
//		*reflect_coef = (temp - 1 / temp + 1)	// temp is VSWR from antenna datasheet

//		//fread(&temp, 4, 1, f);
//		//	need to put something in here to read the polarization information with which we calculate PLF

//		for( i = 0; i < STEPS; i++)
//		{
//			for( j = 0; j < STEPS; j++)
//			{
//				fread(&temp, 4, 1, f);
//				out_gain[i][j] = temp;
//			}
//		}

//		fclose(f);
//	}

//}
//*/

//void antennaRead( const char file_name[], float t_wavelength, float t_power, float r_sens,
//				  float reflect_coef, float *pol_vec, float axial_ratio,
//				  float *out_gain )
//{
//	TFileHandle file_handle;	// create a file handle variable 'myFileHandle'
//	TFileIOResult io_result;	// create an IO result variable 'IOResult'
//	int file_size;

//	OpenRead(file_handle, io_result, file_name, file_size);    // open for read, size is 'myFileSize'
//	ReadFloat(file_handle, io_result, t_wavelength);
//	ReadFloat(file_handle, io_result, t_power);
//	ReadFloat(file_handle, io_result, r_sens);

//	float temp;
//	ReadFloat(file_handle, io_result, temp);
//	reflect_coef = (temp - 1 / temp + 1);	// temp is VSWR from antenna datasheet

//	ReadFloat(file_handle, io_result, pol_vec[0]);
//	ReadFloat(file_handle, io_result, pol_vec[1]);
//	ReadFloat(file_handle, io_result, pol_vec[2]);
//	ReadFloat(file_handle, io_result, axial_ratio);

//	for(int i = 0; i < STEPS; i++)
//	{
//		for(int j = 0; j < STEPS; j++)
//		{
//			ReadFloat(file_handle, io_result, out_gain[i][j]);
//		}
//	}

//}


//static float PLF( const float *a1_pol_in_a2_vec, const float a1_axial_ratio,
//		   const float *a2_pol_vec, const float a2_axial_ratio )
//{
//	// Poincaire Sphere representations:

//	float a1_latitude, a2_latitude;

//	if (a1_axial_ratio == 0)	{a1_latitude = 0;}
//	else 						{a1_latitude = 2 * atan(1/a1_axial_ratio);}

//	if (a2_axial_ratio == 0)	{a2_latitude = 0;}
//	else						{a2_latitude = 2 * atan(1/a2_axial_ratio);}

//	float temp, longitude;
//	temp = (a1_pol_in_a2_vec[0] * a2_pol_vec[0] +
//			a1_pol_in_a2_vec[1] * a2_pol_vec[1] +
//			a1_pol_in_a2_vec[2] * a2_pol_vec[2]);

//	longitude = 2 * acos(temp);

//	float great_circle;
//	great_circle = acos(cos(a1_latitude - a2_latitude) * cos(longitude));

//	temp = cos(great_circle / 2);
//	return temp * temp;
//}

/////////////////////////////////////

task linearactuator()
{
	float distance,counts;
	int direction;
	// Say I would like to have it move for 2 seconds.
	// With full power, 1cm is covered in 1 sec.
	// say for turning right we need to move up and for turning left we need to move right
	// for up -> it becomes more negative and for down - > it becomes less negative

	distance = exp_time/2; // need to come back to center
	direction=1;
	if(direction==1)
	{
		counts = distance * 100 / 5 + 100;
		while(nMotorEncoder[motorC] > -counts)
		{
			motor[motorC]=100;
			nxtDisplayTextLine(7,"%d",nMotorEncoder[motorC]);
		}
		while(nMotorEncoder[motorC] < -100)
		{
				motor[motorC]=-100;
	  		nxtDisplayTextLine(7,"%d",nMotorEncoder[motorC]);
		}
		motor[motorC] = 0; //turn both motors off
	}
	else
	{
		counts = 100 - distance * 100 / 5;
		while(nMotorEncoder[motorA] < -counts)
		{
			motor[motorC]=-100;
			nxtDisplayTextLine(7,"%d",nMotorEncoder[motorC]);
		}
		while(nMotorEncoder[motorC]>-100)
		{
			motor[motorC]=100;
			nxtDisplayTextLine(7,"%d",nMotorEncoder[motorC]);
		}
		motor[motorC] = 0;
	}

}
task smoothturn()
{
float vel=418,radius,ang_vel,power,speed1,speed2,lambda,distance,theta,direction=0;
// float exp_time;
// vel. = 417.6cm/min
// lamda = rate exp. parameter
// speed1 - wheelA
// speed2 - wheelB
// exp_time - exponential time to run
// distance - distance covered

float N1,N2;
// N1 and N2 - encoder counts
// theta- angle with the center
// Rg - dis between prevx and newx
// R angle, (180 - theta) /2
lambda=30/60;
float length,x,y,prev_theta=0,cumm_x,cumm_y,x_prime,y_prime,x_pprime,y_pprime;//thetasend=0;
int max=30,min=5.8,dir,p=0;//,sensor_reading1,sensor_reading2;//stopsignal=0,stopsignal1=0,count=0;
// max and min - radius max and min
//lambda=1/60;
 int i=1;
	while(1)
	{
	exp_time=abs((rand()%100+1));
	exp_time=abs(log(1-exp_time/100));
	exp_time=60*exp_time/30; // exp_time/lambda

	nxtDisplayTextLine(4, "time %f",exp_time);  // Display the text on line number 4 of 8 on the LCD

	dir=abs(rand()%(100));
	radius=abs((rand() % (max-min))) + min;
	ang_vel=vel/(radius+5.8);
	power=ang_vel*60/72;	// here we have considered max. angular vel. as 72rpm for power = 60
//the motor specifications are 120 rpm for 100 power, but with experiment i found that it will
//work fine if i consider 60 power as the maximum with ang. vel (max)=72 rpm
  distance=abs(ang_vel*2*PI*2.8*exp_time/60); // 2.8 is radius of the wheel
  nxtDisplayTextLine(1, "distance %f",distance);  // Display the text on line number 4 of 8 on the LCD
	theta=distance/(radius+5.8) * 360/(2*PI);
  nxtDisplayTextLine(2, "theta %f",theta);  // Display the text on line number 4 of 8 on the LCD
  length=abs(2*(radius+2.5)*sinDegrees(theta/2));
  nxtDisplayTextLine(3,"Length %f",length);

  // theta
  // 11.6 cm is the width of the robot
	if (dir<=50)
	{
    p=-1;
		speed1=power;
		speed2=speed1*(radius-5.8)/(radius+5.8);
		N1=distance*360/(2*PI*2.8);	// encoder counts, now in terms of exponential time
	}

	else
	{
		p=1;
		speed2=power;
		speed1=speed2*(radius-5.8)/(radius+5.8);
		N2=distance*360/(2*PI*2.8);	// encoder counts, now in terms of exponential time
		N1=speed1/speed2 * N2;
	}

  nMotorEncoder[motorA]=0;
  nMotorEncoder[motorB]=0;
  if(speed1>speed2)
  {
  theta=theta;
  direction_actuator=1;
	StartTask(linearactuator);
 	while(nMotorEncoder[motorA] < (N1) && direction==0 )
 	{
 			motor[motorA]=speed1;
			motor[motorB]=speed2;
	}
 }
 else
{
		theta=-theta;
		direction_actuator=-1;
	StartTask(linearactuator);
	while(nMotorEncoder[motorB] < (N2) && direction==0 )
 	{
			motor[motorA]=speed1;
			motor[motorB]=speed2;
 	}
 }

 nMotorEncoder[motorA]=0;
 nMotorEncoder[motorB]=0;
StopTask(linearactuator);

 if(i==1)
	{
		x=length*cosDegrees(theta/2);
		y=length*sinDegrees(theta/2);
	}
	else
	{
		if(prev_theta>0)
  	{
  		x_prime=x*cosDegrees(abs(prev_theta)) + y*sinDegrees(abs(prev_theta));
  		y_prime=-x*sinDegrees(abs(prev_theta))+ y*cosDegrees(abs(prev_theta));
		}
  	else
  	{
    	x_prime=x*cosDegrees(abs(prev_theta))-y*sinDegrees(abs(prev_theta));
    	y_prime=x*sinDegrees(abs(prev_theta))+y*cosDegrees(abs(prev_theta));
		}
		x_pprime=length*cosDegrees(theta/2);
		y_pprime=length*sinDegrees(theta/2);

		cumm_x=x_prime+x_pprime;
		cumm_y=y_prime+y_pprime;

		if(prev_theta>0)
  	{
  	  x=cumm_x*cosDegrees(abs(prev_theta))-cumm_y*sinDegrees(abs(prev_theta));
  		y=cumm_x*sinDegrees(abs(prev_theta))+cumm_y*cosDegrees(abs(prev_theta));
		}
  		else
		{
			x=cumm_x*cosDegrees(abs(prev_theta))+cumm_y*sinDegrees(abs(prev_theta));
  		y=-cumm_x*sinDegrees(abs(prev_theta))+cumm_y*cosDegrees(abs(prev_theta));
		}
	}

	nxtDisplayTextLine(4,"X %f",x);
	nxtDisplayTextLine(5,"Y %f",y);

 prev_theta=prev_theta+theta;

if(prev_theta>180 && prev_theta<360)
	prev_theta=prev_theta-360;
else if(prev_theta>360)
	prev_theta=prev_theta-360;
else if(prev_theta<-360)
	prev_theta=-prev_theta+360;
else if(prev_theta<-180 && prev_theta>-360)
	prev_theta=-prev_theta+360;
else
	prev_theta=prev_theta;

	motor[motorA]=0;
	motor[motorB]=0;
  direction=0;
  eraseDisplay();

  }
}

//int matInverse( const float a_mat[3][4], float a_inv_mat[3][4]);
//void matInverse(const float *a_mat, float *a_inv_mat);
task main()
{
nxtEnableHSPort();                                //Enable High Speed Port #4
nxtSetHSBaudRate(9600);                           //Xbee Default Speed
nxtHS_Mode = hsRawMode;                           //Set to Raw Mode (vs. Master/Slave Mode)

int stopsignal=0, stopsignal1=0;

char messagereceive;
char messagesend;
string xRec,yRec;
	tMotor testMotor = motorC;
// back to Original Position
  PlaySound(soundBeepBeep);
  nxtDisplayBigTextLine(3, "Retract");
  FLACretractLA(testMotor, 100);
  while(!isDone(testMotor)) wait1Msec(50);
  wait1Msec(200);

  // set count to zero
	// and move it to the center
  nMotorEncoder[motorC] = 0;
  while(nMotorEncoder[motorC] > -100)
	{
	motor[motorC]=100;
	nxtDisplayTextLine(1,"%d",nMotorEncoder[motorC]);
	}
	motor[motorC]=0;
	nxtDisplayTextLine(4,"At the center");
	wait1Msec(2000);
	eraseDisplay();

//	wait1Msec(1000);

StartTask(smoothturn);
while(stopsignal==0 && stopsignal==0)
{
	if(nxtGetAvailHSBytes())                        //Check to see if we have any data coming in
	{
			nxtReadRawHS(&messagereceive, 2);
		  if(messagereceive=='f')
		  {
		   stopsignal=1;
		   messagesend='c';
	     nxtDisplayTextLine(7,"1st group");
			 nxtWriteRawHS(&messagesend, 1);                      //Write the data (paras: char to send, length of data)
		   break;
	    }
	    if(messagereceive=='c')
	    {
	     stopsignal1=1;
	     messagesend='f';
	     nxtDisplayTextLine(7,"2nd group");
			 nxtWriteRawHS(&messagesend, 1);                      //Write the data (paras: char to send, length of data)
	     break;
	    }
  }
}
int count=0;
//wait1Msec(4000);
while(count==0)
{
  string msend="found";
 	ReceiveString(xRec);
	ReceiveString(yRec);

	wait1Msec(2000);                                //Half Second Delay (can speed this up)
  eraseDisplay();
	SendString(msend);

	nxtDisplayTextLine(1,"X-Cor:%s",xRec);
	nxtDisplayTextLine(2,"Y-Cor:%s",yRec);
  wait1Msec(2000);                                //Half Second Delay (can speed this up)

	SendString(xRec);
	wait1Msec(500);
	SendString(yRec);
  count=count+1;
}
nxtDisableHSPort();                               //Disable HS Port #4

}
