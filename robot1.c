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

char messagereceive;
char messagesend='f';
string xRec,yRec,xSend,ySend,my_message;
int turnA,stopsignal=0,stopsignal1=0,count,newcount;
float travelled,theta=0,x=0,y=0,newx,newy,angle;
int num,rotation,distance;
int times=0;
//ubyte send,incomingData;

// turn - 1 -> left, -1->right
while(SensorValue[lightSensor] < 35 && stopsignal==0 && stopsignal1==0)
{
	times=times+1;
	wait1Msec(1000);
	if(nxtGetAvailHSBytes())                        //Check to see if we have any data coming in
	{
		nxtReadRawHS(&messagereceive, 2);
		//ReceiveString(messagereceive);
	//	nxtDisplayTextLine(7,"receive %s",messagereceive);
			if(messagereceive=='f')
		  {
			  //If we actually got data, display it to the LCD
// 		  nxtDisplayTextLine(6, "Receive: %c", messagereceive);
		   stopsignal=1;
		   break;
	    }
  }
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


if (stopsignal==1)
{
	eraseDisplay();
	ReceiveString(xRec);
	ReceiveString(yRec);
	newx=atof(xRec);
	newy=atof(yRec);
	nxtDisplayTextLine(1,"Received");
	nxtDisplayTextLine(2, "New x: %f", newx);
	nxtDisplayTextLine(3, "New y: %f", newy);
// send coordinates to other robot
	my_message='f';
	cCmdMessageWriteToBluetooth(my_message,strlen(my_message),1);
  // sendMessageWithParm(x,y);
  //sendMessage(y);
wait1Msec(3000);

	cCmdMessageWriteToBluetooth(xRec,strlen(xRec),2);
	nxtDisplayTextLine(2, "New x: %s", xSend);

  wait1Msec(500);
	cCmdMessageWriteToBluetooth(yRec,strlen(yRec),1);
	nxtDisplayTextLine(3, "New y: %s", ySend);


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
	turnA=1;
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
else if(stopsignal1==1)
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

	messagesend='f';
	 nxtWriteRawHS(&messagesend, 1);                      //Write the data (paras: char to send, length of data)
	 wait1Msec(3000);                                //Half Second Delay (can speed this up)

	SendString(xRec);
	 wait1Msec(500);
	 SendString(yRec);

    nxtDisplayTextLine(1,"Received Bluetooth");
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
	turnA=1;
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

		messagesend='f';
	 nxtWriteRawHS(&messagesend, 1);                      //Write the data (paras: char to send, length of data)
	 wait1Msec(3000);                                //Half Second Delay (can speed this up)

SendString(xSend);
	 wait1Msec(500);
	 SendString(ySend);

}
wait1Msec(3000);
nxtDisableHSPort();                               //Disable HS Port #4
}
