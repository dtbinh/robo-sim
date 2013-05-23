#pragma config(Sensor, S1,     sonarSensor1,         sensorSONAR)
#pragma config(Sensor, S2,     sonarSensor2,         sensorSONAR)


#include "XbeeTools.h"

task smoothturn()
{
float vel=418,radius,ang_vel,power,speed1,speed2,exp_time,lambda,distance,theta,direction=0;
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
float length,x,y,prev_theta=0,cumm_x,cumm_y,x_prime,y_prime,x_pprime,y_pprime,thetasend=0;
int max=30,min=5.8,dir,p=0,sensor_reading1,sensor_reading2;//stopsignal=0,stopsignal1=0,count=0;
// max and min - radius max and min
//lambda=1/60;
 int i=1;
	while(i<10)
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
  thetasend=thetasend+theta;


if(thetasend>180 && thetasend<360)
	thetasend=thetasend-360;
else if(prev_theta>360)
	thetasend=thetasend-360;
else if(thetasend<-360)
	thetasend=-thetasend+360;
else if(thetasend<-180 && thetasend>-360)
	thetasend=-thetasend+360;
else
	thetasend=thetasend;


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

 	while(nMotorEncoder[motorA] < (N1) && direction==0 )
 	{
 		sensor_reading1=SensorValue(sonarSensor1);
 		sensor_reading2=SensorValue(sonarSensor2);

//		nxtDisplayTextLine(6,"Block Left %d",sensor_reading1);
//		nxtDisplayTextLine(7,"Block Right%d",sensor_reading2);

		if(sensor_reading1<100 || sensor_reading2<100)
		{
			direction=1;
			break;
		}
		else
		{
			motor[motorA]=speed1;
			motor[motorB]=speed2;
		}
 	}
 }
 else
{
		theta=-theta;

	while(nMotorEncoder[motorB] < (N2) && direction==0 )
 	{
 		sensor_reading1=SensorValue(sonarSensor1);
 		sensor_reading2=SensorValue(sonarSensor2);


	//	nxtDisplayTextLine(6,"Block Left %d",sensor_reading1);
	//	nxtDisplayTextLine(7,"Block Right%d",sensor_reading2);

		if(sensor_reading1<100 && sensor_reading2<100)
		{
			direction=2;
			break;
		}
		else
		{
			motor[motorA]=speed1;
			motor[motorB]=speed2;

		}
 	}
 }
 nMotorEncoder[motorA]=0;
 nMotorEncoder[motorB]=0;

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


// if(direction==1)
// {
//    sensor_reading1=SensorValue(sonarSensor1);
//    sensor_reading2=SensorValue(sonarSensor2);

//    nxtDisplayTextLine(6,"Block Left %d",sensor_reading1);
//		nxtDisplayTextLine(7,"Block Right%d",sensor_reading2);

//		while(sensor_reading1<100 && sensor_reading2<100)
//  	{
// 			sensor_reading1=SensorValue(sonarSensor1);
// 			sensor_reading2=SensorValue(sonarSensor2);

//			radius= min;
//			ang_vel = vel/(radius+5.8);
//			power=ang_vel*60/72;	// here we have considered max. angular vel. as 72rpm for power = 60
//		  speed1=power;
//		  speed2=speed1*(radius-5.8)/(radius+5.8);
//			motor[motorA]=speed1;
// 		  motor[motorB]=speed2;
// 		}
// }
//if(direction==2)
//{   sensor_reading1=SensorValue(sonarSensor1);
//    sensor_reading2=SensorValue(sonarSensor2);

//    nxtDisplayTextLine(6,"Block Left %d",sensor_reading1);
//		nxtDisplayTextLine(7,"Block Right%d",sensor_reading2);

//		while(sensor_reading1<100 && sensor_reading2<100)
//  	{	sensor_reading1=SensorValue(sonarSensor1);
//  		sensor_reading2=SensorValue(sonarSensor2);

//		  radius = min;
//			ang_vel=vel/(radius+5.8);
//			power=ang_vel*60/72;	// here we have considered max. angular vel. as 72rpm for power = 60
//		  speed1=power;
//		  speed2=speed1*(radius-5.8)/(radius+5.8);
//		  motor[motorB]=speed1;
// 		  motor[motorA]=speed2;
// 		}
//}
	motor[motorA]=0;
	motor[motorB]=0;
  direction=0;
  wait1Msec(4000);
  eraseDisplay();
i=i+1;

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
