#pragma config(Sensor, S1,     lightSensor,    sensorLightActive)
#include "XbeeTools.h"

int turn(int rotation,int direction)
{
	nMotorEncoder[motorA]=0;
	nSyncedMotors = synchAB; //motor A is the master, motor B is the slave
  nSyncedTurnRatio = -100; //motors move in opposite directions of each other

  while(abs(nMotorEncoder[motorA]) < (rotation) && SensorValue[lightSensor] < 35)
	{
	motor[motorA]=direction*20;
//	nxtDisplayTextLine(5, " %d",nMotorEncoder[motorA]);  // Display the text on line number 4 of 8 on the LCD
	}
	motor[motorA]=0;
	return nMotorEncoder[motorA];
}

int move(int distance)
{
	nMotorEncoder[motorA]=0;
  nSyncedTurnRatio = 100; //motors move in opposite directions of each other

  while(nMotorEncoder[motorA] < distance && SensorValue[lightSensor] < 35)
	{
	motor[motorA]=30;
	}
	motor[motorA]=0;
	return nMotorEncoder[motorA];
}

task main()
{
//InitRS485();

nxtEnableHSPort();                                //Enable High Speed Port #4
nxtSetHSBaudRate(9600);                           //Xbee Default Speed
nxtHS_Mode = hsRawMode;                           //Set to Raw Mode (vs. Master/Slave Mode)

//char messagereceive,messagesend='f';
string my_message;
string xRec,yRec,xSend,ySend;
int turnA,count,newcount,stopsignal1=0;
float travelled,theta=0,x=60,y=0,newx,newy,angle;
int num,rotation,distance;
int times=0;
//ubyte send,incomingData;

// turn - 1 -> left, -1->right
while(SensorValue[lightSensor] < 35 && stopsignal1==0)
{
	times=times+1;
	wait1Msec(1000);
cCmdMessageRead(my_message,1,1);
if(my_message == "f")
	{
		stopsignal1=1;
		break;
	}


	num=abs(rand()%100+1);
	rotation=abs(rand()%180+1);
	distance=abs(rand()%360);
	if (num<=50)	turnA=1;
	else		turnA=-1;

  count=turn(rotation,turnA);
  travelled=move(distance);//* 2*PI*2.8;
	travelled=travelled*2*PI*2.8/360;
  if (times==1)
		theta=90+count/2;
	else
		theta=theta+count/2;

	if(theta>180)
		theta=theta-360;
	else if (theta<-180)
		theta=360+theta;
	else
		theta=theta;

	x+=travelled*cosDegrees(theta);
	y+=travelled*sinDegrees(theta);
	nxtDisplayTextLine(1, "Angle %f",theta);  // Display the text on line number 4 of 8 on the LCD
	nxtDisplayTextLine(2, "X-loca %f",x);  // Display the text on line number 4 of 8 on the LCD
  nxtDisplayTextLine(3, "Y- loca %f",y);  // Display the text on line number 4 of 8 on the LCD
}

if(stopsignal1==1)
{
	eraseDisplay();
	nxtDisplayTextLine(6,"%s",my_message);
 //wait1Msec(1000);
	while(1)
	{
	cCmdMessageRead(xRec,10,2);

//	wait1Msec(1000);
  cCmdMessageRead(yRec,10,1);

newx=atof(xRec);
newy=atof(yRec);

if (newx!=0 && newy!=0)
	break;
}
 //   newx = messageParm[0];
  //  newy = messageParm[1];

    nxtDisplayTextLine(1,"Received");
		nxtDisplayTextLine(2, "New x: %f", newx);
		nxtDisplayTextLine(3, "New y: %f", newy);
  	distance=sqrt((x-newx)*(x-newx) + (y-newy)*(y-newy));
	  distance=abs(distance*360/(2*PI*2.8));
  	angle=atan((y-newy)/(x-newx))/(2*PI) * 360;

  	nxtDisplayTextLine(4, "Angle: %f", angle);
if(x<newx)
{
		newcount=abs((0-theta)*2);
if (theta<0)
	turnA=1;
else
	turnA=-1;
count=turn(newcount,turnA);
if(y>newy)
{
	turnA=-1;
	count=turn(abs(angle)*2,turnA);
}
else
{
	turnA=+1;
	count=turn(abs(angle)*2,turnA);
}

}
 else
{
  if(theta<0)
{		newcount=abs((-180-theta)*2);
  		turnA=-1;
}
 else
{
	newcount=abs((180-theta)*2);
	turnA=1;
}
count=turn(newcount,turnA);
if(y>newy)
{
	turnA=+1;
	count=turn(abs(angle)*2,turnA);
}
else
{
	turnA=-1;
	count=turn(abs(angle)*2,turnA);
}

}

nxtDisplayTextLine(5, "distance: %d", distance);
	travelled=move(distance);

	wait1Msec(5000);


}
else
{
	 eraseDisplay();
//wait1Msec(3000);

	my_message='f';
	cCmdMessageWriteToBluetooth(my_message,strlen(my_message),1);
  // sendMessageWithParm(x,y);
  //sendMessage(y);
 	 sprintf(xSend,"%f",x);
	 sprintf(ySend,"%f",y);
wait1Msec(3000);

	cCmdMessageWriteToBluetooth(xSend,strlen(xSend),2);
	nxtDisplayTextLine(2, "New x: %s", xSend);

  wait1Msec(500);
	cCmdMessageWriteToBluetooth(ySend,strlen(ySend),1);
	nxtDisplayTextLine(3, "New y: %s", ySend);

}
wait1Msec(3000);
nxtDisableHSPort();                               //Disable HS Port #4
}
