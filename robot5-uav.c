#include "XbeeTools.h"

task main()
{

nxtEnableHSPort();                                //Enable High Speed Port #4
nxtSetHSBaudRate(9600);                           //Xbee Default Speed
nxtHS_Mode = hsRawMode;                           //Set to Raw Mode (vs. Master/Slave Mode)


float vel=417.6,radius,ang_vel,power,speed1,speed2,xcenter,ycenter,x=0,y=0,phi=0,exp_time,lambda,distance;
// vel. = 417.6cm/min
// lamda = rate exp. parameter
// speed1 - wheelA
// speed2 - wheelB
// exp_time - exponential time to run
// distance - distance covered

float N1,N2,theta,Rg,R;
// N1 and N2 - encoder counts
// theta- angle with the center
// Rg - dis between prevx and newx
// R angle, (180 - theta) /2
lambda=30/60;
int max=30,min=5.8,dir,s=0,p=0,stopsignal=0,count=0;
// max and min - radius max and min
//lambda=1/60;
char messagereceive;
char messagesend;
string xRec,yRec;
while(stopsignal==0)
{

	if(nxtGetAvailHSBytes())                        //Check to see if we have any data coming in
	{
			nxtReadRawHS(&messagereceive, 2);
		  if(messagereceive=='f')
		  {
			  //If we actually got data, display it to the LCD
// 		  nxtDisplayTextLine(6, "Receive: %c", messagereceive);
		   stopsignal=1;
		   break;
	    }
  }

	exp_time=abs((rand()%100+1));
	exp_time=abs(log(1-exp_time/100));
	exp_time=60*exp_time/30; // exp_time/lambda
	if(exp_time>9)
		continue;

	nxtDisplayTextLine(4, "time %f",exp_time);  // Display the text on line number 4 of 8 on the LCD

	dir=abs(rand()%(100));
	radius=abs((rand() % (max-min))) + min;
	ang_vel=vel/(radius+5.8);
	power=ang_vel*60/72;	// here we have considered max. angular vel. as 72rpm for power = 60
	// the motor specifications are 120 rpm for 100 power, but with experiment i found that it will
// work fine if i consider 60 power as the maximum with ang. vel (max)=72 rpm
 distance=abs(ang_vel*2*PI*2.8*exp_time/60); // 2.8 is radius of the wheel
 nxtDisplayTextLine(1, "distance %f",distance);  // Display the text on line number 4 of 8 on the LCD

// 11.6 cm is the width of the robot
	if (dir<=50)
	{
		p=-1;
		speed1=power;
		speed2=speed1*(radius-5.8)/(radius+5.8);
		N1=distance*360/(2*PI*2.8);	// encoder counts, now in terms of exponential time
 		if(s==1)
 			phi=180;
	}

	else
	{
		p=1;
		speed2=power;
		speed1=speed2*(radius-5.8)/(radius+5.8);
		N2=distance*360/(2*PI*2.8);	// encoder counts, now in terms of exponential time
		N1=speed1/speed2 * N2;
		if(s==1)
			phi=0;
 	}

 	nxtDisplayTextLine(5, "encoder %f",N1);  // Display the text on line number 4 of 8 on the LCD
	nxtDisplayTextLine(6, "speed1 %f",speed1);  // Display the text on line number 4 of 8 on the LCD
	nxtDisplayTextLine(7, "speed2 %f",speed2);  // Display the text on line number 4 of 8 on the LCD

 nMotorEncoder[motorA]=0;
 nMotorEncoder[motorB]=0;

 while(nMotorEncoder[motorA] < (N1) )
 {
	motor[motorA]=speed1;
	motor[motorB]=speed2;
 }
	motor[motorA]=0;
	motor[motorB]=0;

	xcenter=x+p*radius*cosDegrees(phi);
	ycenter=y-p*radius*sinDegrees(phi);

	theta=distance/(radius+5.8) * 360/(2*PI);

	if(theta>180)
	{
		theta=360-theta;
		R=-(180-theta)/2;
	}
	else
	{
		theta=theta;
		R=(180-theta)/2;
	}

	Rg=abs(sinDegrees(theta)*(radius)/sinDegrees(R));

if (s==1)
{
	x=x+p*Rg*cosDegrees(R);
	y=y+p*Rg*sinDegrees(R);
}
else
{
	x=x+p*Rg*cosDegrees(phi-R);
	y=y+p*Rg*sinDegrees(phi-R);
}
	s=s+1;
	nxtDisplayTextLine(1, " x-center %f",xcenter);  // Display the text on line number 4 of 8 on the LCD
	nxtDisplayTextLine(2, " y-center %f",ycenter);  // Display the text on line number 4 of 8 on the LCD
	nxtDisplayTextLine(3, " x-cord %f",x);  // Display the text on line number 4 of 8 on the LCD
	nxtDisplayTextLine(4, " y-cord %f",y);  // Display the text on line number 4 of 8 on the LCD
	nxtDisplayTextLine(5, " theta %f",theta);  // Display the text on line number 4 of 8 on the LCD
//  wait1Msec(5000);

  phi=phi-theta;
count=count+1;
}
//wait1Msec(4000);

if(stopsignal==1)
{
  eraseDisplay();
 	ReceiveString(xRec);
	ReceiveString(yRec);

	nxtDisplayTextLine(1,"X-Cor:%s",xRec);
	nxtDisplayTextLine(2,"Y-Cor:%s",yRec);

	wait1Msec(500);                                //Half Second Delay (can speed this up)

}
nxtDisableHSPort();                               //Disable HS Port #4

	messagesend='f';
	nxtWriteRawHS(&messagesend, 1);                      //Write the data (paras: char to send, length of data)
	wait1Msec(3000);                                //Half Second Delay (can speed this up)

SendString(xRec);
wait1Msec(500);
SendString(yRec);

nxtDisableHSPort();                               //Disable HS Port #4

}
