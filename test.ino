#include <NewPing.h>
#include "MotorControllerMaster.h"
#include <AccelStepper.h>
#include <cmath>
#include <DistanceGP2Y0A21YK.h>
#include <DistanceGP2Y0A21YK_LUTs.h>
#include <Encoder.h>

#define SAFE_DISTANCE 30

const int trigger_pin = 22;
const int echo_pin =  23;
const int front_ir_pin = A3;
const int rear_ir_pin = A4;
const int stepper_step = 2;
const int stepper_dir = 3;
// const int l_encoder_pin1 = ;
// const int l_encoder_pin2 = ;
// const int r_encoder_pin1 = ;
// const int r_encoder_pin2 = ;
 
int lasthb = 0;
int lastRightTurn = 0;
int lastEncoderSample = 0;
int lastPing = 0;
int lastir = 0;

long l, r, reference_l, reference_r , encoder_l, encoder_r= 0;

unsigned int frontWallDistance;
unsigned int frontDist;
unsigned int rearDist;
unsigned int sideWallDistance;

const int frontIRXpos = 3;
const int rearIRXpos = 3;
const int frontIRYpos = 5.5;
const int rearIRYpos  = -5.5;

//int irMap[2][11] ={{5,6,7,8,9,10,11,12,13,14,15},{399,340,287,260,240,212,189,175,165,153,146}};
double sideWallAngle;

const int OpenIrValue = 50;
const int threshHold = 20;


enum dirveStates{goStraight, turnLeft_90, turnRight_90, turnToOpenArea, brake} driveState;

bool facingCliff, nearFrontWall, rightIsOpen, atCliff, getReferencePos , stop_move = false;



NewPing sonar(trigger_pin, echo_pin);
MotorControllerMaster c;
DistanceGP2Y0A21YK frontIR;
DistanceGP2Y0A21YK rearIR;
AccelStepper stepper(AccelStepper::DRIVER ,stepper_step, stepper_dir );
RegulatedMotor m1();
//Encoder encl(l_encoder_pin1, l_encoder_pin2);
//Encoder encr(r_encoder_pin1, r_encoder_pin2);

void setup()
{	
	//Wire.begin();
	c.begin();

	frontIR.begin(front_ir_pin);
	rearIR.begin(rear_ir_pin);

	Serial.begin(9600);
	delay(1000);
	c.setAcceleration(400,600,400,600);
	c.brake();
	c.goVelocity(100, 0);
	delay(200);
	driveState = goStraight;
}

void loop()
{	
	
	sendHb();
	//checkCliff();
	pingSonar();
	//checkFlame();
	getReferencePosition();
	Go();
	//Serial.println(reference_r - r);

	if(facingCliff || nearFrontWall)
	{
		//turn left 90 degrees
		//Serial.println("set to turn state");
      if(stop_move){
                c.goVelocity(0,0);
                delay(3000);
                stop_move = false;
      }
              //  if(c.isStandby())
             // {
                 driveState = turnLeft_90;
              //}
		//flag set back to false in Go method!
	}
	if(rightIsOpen && !atCliff)
	{
		//turn to open area
		//Serial.println("wrong");
		driveState = turnToOpenArea;
		//flag set back to false in Go method!
	}
	
	
	if(millis() - lastir > 50)
	{
		Serial.println("readIR");
		lastir = millis();
		
		getSideDistance();
		


		frontDist = frontIR.getDistanceCentimeter();
		rearDist = rearIR.getDistanceCentimeter();
		
		if(frontDist > OpenIrValue && rearDist > OpenIrValue)
		{
			if(!rightIsOpen)
			{
				getReferencePos = true;
			}

		    rightIsOpen = true;
		}

		else
		atCliff = false;

	}

	if(sideWallDistance - threshHold > 5)
	{
		// turn left
		//c.goVelocity(100, 2);
        c.goVelocity(100, map(sideWallDistance - threshHold, 0,200, 0,10));
	}
	
	else if (sideWallDistance - threshHold < -5)
	{	
		//turn right
		c.goVelocity(100, -2);
        c.goVelocity(100,map(- sideWallDistance + threshHold, 0,200, -10 , 0));
	}
	else
	{
		//go straight
		driveState = goStraight;
	}

 // c.goVelocity(100,map(sideWallDistance - threshHold, -500,500, -50,50));

}

void pingSonar()
{
	if (millis() - lastPing > 100)
	{
		//frontWallDistance = sonar.convert_cm(echoTime);
		lastPing = millis();
		sonar.ping_timer(echoCheck);
	}		
}

void echoCheck()
{	
	if(sonar.check_timer())
	{
		frontWallDistance = sonar.ping_result / US_ROUNDTRIP_CM;
		
		if(frontWallDistance < SAFE_DISTANCE )//and if flame is not close by!! 
		{	
			if(nearFrontWall == false)
			{	
				stop_move = true;
				getReferencePos = true;
			}
			nearFrontWall = true;
		}	
	}	
}

//!!!!!!!!!!not complete
void checkCliff()
{
	facingCliff = false;
	atCliff = false;
}

void sendHb()
{
	if (millis() - lasthb > 50){
				//Serial.println("inhb");
		c.heartbeat();
		lasthb = millis();	
	}
}

void getReferencePosition()
{
	if(getReferencePos)
	{
		//Serial.println("getref");
		c.getEncoder(&l, &r);
		reference_l = l;
		reference_r = r;
		getReferencePos = false;
	} 
}
void checkFlame()
{

}

void getCurrentPosition()
{
	Serial.println("getcur");
	if(millis() - lastEncoderSample > 20 ) 
    {
		c.getEncoder(&l, &r);	
		lastEncoderSample = millis();	
	}
}

void Go()
{
	switch (driveState) {
	    case goStraight:
	    //Serial.println("straight");
		    c.goVelocity(100, 0);
	    	break;
	    
	    case turnLeft_90:
	    	//Serial.println("im turning");

	    	c.goVelocity(0, 20);          
        	getCurrentPosition();
			if (r - reference_r  > 80)
			{
			//Serial.println("complete turn");
				 c.goVelocity(0,0);
                 delay(1000);
				driveState = goStraight;
				nearFrontWall = false;
				facingCliff = false;
			}	
	      	break;
	    
	    case turnRight_90:
	    	c.goVelocity(0, -90);          
        	getCurrentPosition();
			if (reference_r - r > 85)
			{
				driveState = goStraight;
				rightIsOpen = false;
			}
			break;

		case turnToOpenArea:
			c.goVelocity(100, 0);
			getCurrentPosition();
			if(l - reference_l > 200) //20cm
			{
				getReferencePos = true;
				driveState = turnRight_90;
			}

			break;
		
		case brake:
			c.brake();
			break;

	    default:
	    	c.brake();
	}
}
	
void getSideDistance()

	{
		getSideAngle();
		int y1 = frontDist*cos(-sideWallAngle) + frontIRXpos*cos(sideWallAngle) + frontIRYpos*sin(sideWallAngle);
		int y2 = rearDist*cos(-sideWallAngle) + rearIRXpos*cos(sideWallAngle) + rearIRYpos*sin(sideWallAngle);
		
		sideWallDistance = (y1 + y2 )/2;
	}


void getSideAngle() 
{
	//convert ir value to distance
	sideWallAngle = atan((frontDist - rearDist + frontIRXpos -rearIRXpos)/(rearDist*sin(0)- frontDist*sin(0) + rearIRYpos - frontIRYpos));
}