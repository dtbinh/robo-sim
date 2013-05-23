//#pragma config(Sensor, S1,     sonarSensor1,         sensorSONAR)
//#pragma config(Sensor, S2,     sonarSensor2,         sensorSONAR)

#include "XbeeTools.h"
#include "C:\Program Files (x86)\Robomatter Inc\ROBOTC Development Environment\Sample Programs\NXT\3rd Party Sensor Drivers\drivers\firgelli-linearact.h"
#include "antenna_math.c"
#include "antenna.c"

float exp_time;
int direction_actuator;
float t_wavelength, t_power,r_sens,reflect_coef, axial_ratio;
fvec3 pol_vec;
fmatdata out_gain;

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
task main()
{
nxtEnableHSPort();                                //Enable High Speed Port #4
nxtSetHSBaudRate(9600);                           //Xbee Default Speed
nxtHS_Mode = hsRawMode;                           //Set to Raw Mode (vs. Master/Slave Mode)

int stopsignal=0, stopsignal1=0;

char messagereceive;
char messagesend;
string xRec,yRec;
//	tMotor testMotor = motorC;
//// back to Original Position


//	PlaySound(soundBeepBeep);
//  nxtDisplayBigTextLine(3, "Retract");
//  FLACretractLA(testMotor, 100);
//  while(!isDone(testMotor)) wait1Msec(50);
//  wait1Msec(200);

//  // set count to zero
//	// and move it to the center
//  nMotorEncoder[motorC] = 0;
//  while(nMotorEncoder[motorC] > -100)
//	{
//	motor[motorC]=100;
//	nxtDisplayTextLine(1,"%d",nMotorEncoder[motorC]);
//	}
//	motor[motorC]=0;
//	nxtDisplayTextLine(4,"At the center");
//	wait1Msec(2000);
//	eraseDisplay();

//	wait1Msec(1000);
	/////////////////// Check data
string filename="SampleAnt.dat";

antennaRead(filename,t_wavelength,t_power,r_sens,reflect_coef,pol_vec,axial_ratio,out_gain );

fvec3 a1_pos_vec, v1_pos_vec, a2_pos_vec, v2_pos_vec;
ivec3 a1_rot_vec, v1_rot_vec, a2_rot_vec,  v2_rot_vec;

	fvec3 a1_other,a2_other;
float output;
	a1_pos_vec[0]=0;
	a1_pos_vec[1]=0;
	a1_pos_vec[2]=0;

	a2_pos_vec[0]=0;
	a2_pos_vec[1]=0;
	a2_pos_vec[2]=0;

	v1_pos_vec[0]=10;
	v1_pos_vec[1]=0;
	v1_pos_vec[2]=0;

	v2_pos_vec[0]=10;
	v2_pos_vec[1]=0;
	v2_pos_vec[2]=0;

	a1_rot_vec=a1_pos_vec;
  v1_rot_vec=a1_pos_vec;
	a2_rot_vec=a1_pos_vec;
  v2_rot_vec=a1_pos_vec;


	a1_other[0]=axial_ratio;
	a1_other[1]=t_power;
	a1_other[2]=reflect_coef;

	a2_other=a1_other;


output=signalStrength(a1_pos_vec,v1_pos_vec,a2_pos_vec,v2_pos_vec,a1_rot_vec,v1_rot_vec,a2_rot_vec,v2_rot_vec,pol_vec,
					  pol_vec, out_gain, out_gain, t_wavelength, a1_other, a2_other );

eraseDisplay();
nxtDisplayTextLine(3,"%f",output);
wait1Msec(7000);


/////////////////////////////
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
//string filename="SampleAnt.dat";

//antennaRead(filename,t_wavelength,t_power,r_sens,reflect_coef,pol_vec,axial_ratio,out_gain );

//fvec3 a1_pos_vec, v1_pos_vec, a2_pos_vec, v2_pos_vec;
//ivec3 a1_rot_vec, v1_rot_vec, a2_rot_vec,  v2_rot_vec;

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

while(count==1)
{
//	a1_pos_vec[0]=0;
//	a1_pos_vec[1]=0;
//	a1_pos_vec[2]=0;

//	a2_pos_vec[0]=0;
//	a2_pos_vec[1]=0;
//	a2_pos_vec[2]=0;

//	v1_pos_vec[0]=10;
//	v1_pos_vec[1]=0;
//	v1_pos_vec[2]=0;

//	v2_pos_vec[0]=10;
//	v2_pos_vec[1]=0;
//	v2_pos_vec[2]=0;

//	a1_rot_vec=a1_pos_vec;
//  v1_rot_vec=a1_pos_vec;
//	a2_rot_vec=a1_pos_vec;
//  v2_rot_vec=a1_pos_vec;


//	a1_other[0]=axial_ratio;
//	a1_other[1]=t_power;
//	a1_other[2]=reflect_coef;

//	a2_other=a1_other;


//output=isConnected(a1_pos_vec,v1_pos_vec,a2_pos_vec,v2_pos_vec,a1_rot_vec,v1_rot_vec,a2_rot_vec,v2_rot_vec,pol_vec,
//					  pol_vec, out_gain, out_gain, t_wavelength, a1_other, a2_other );

//eraseDisplay();
//nxtDisplayTextLine(3,"%d",output);
//wait1Msec(7000);
count = count+1;
}
nxtDisableHSPort();                               //Disable HS Port #4

}
