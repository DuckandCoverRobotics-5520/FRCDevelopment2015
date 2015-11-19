#include <UsrMath.h>
#include "WPILib.h"
#include "PID.h"
#include <math.h>
#include <algorithm>
#define OPEN 1//breaks the circuit
#define CLOSED 0//completes the circuit
class Robot: public SampleRobot{
	//RobotDrive myRobot; // robot drive system
	PowerDistributionPanel *m_pdp;
	Joystick stick;
	Joystick stick2;
	Joystick stick3;
	//Joystick Launchpad;
	//Joystick Launchpad2;
	Victor Left1;
	Victor Left2;
	Victor Right1;
	Victor Right2;
	Victor Lift1;
	Victor Lift2;
	PIDController PIDLeft;
	PIDController PIDRight;
	DigitalInput Lim_base;
	DigitalInput Lim_top;
	DigitalInput Lim_stack;
	Encoder *LeftEnc;
	Encoder *RightEnc;
//	Encoder *LiftEnc;
	AnalogInput Ultra;
	AnalogInput Pot;

	const int CurrentLimit=0;//will be defined later
	bool smartOverride = false;
	bool DynamicBraking=false;
	bool LiftRunning = false;
	int LoadPosition=0;
	int StackPosition=0;
	float Stop=0.0;
	bool pressed=false;
	int Setpoint=0;
	float JMAX=0.8;
	float JMED=0.4;
	float JMIN=0.2;
public:
	Robot() :
		//	myRobot(0, 1, 2, 3),	// these must be initialized in the same order
	stick(0),		// as they are declared above.
	stick2(1),
	stick3(2),
	Left1(0),
	Left2(1),
	Right1(2),
	Right2(3),
	Lift1(4),
	Lift2(5),
	PIDLeft(0.3, 0.0, 0.0, LeftEnc, &Left1 ,0.005),
	PIDRight(0.3, 0.0, 0.0,RightEnc, &Right1 ,0.005),
	Lim_base(7),//normally closed
	Lim_top(6),//normally closed
	Lim_stack(8),//normally closed
	Ultra(0),
	Pot(1)//probably not in use...



	{
		m_pdp =new PowerDistributionPanel();
		LeftEnc = new Encoder(4, 5, true, Encoder::EncodingType::k4X);//250 ppr *k4x =1000 PPR
		RightEnc =new Encoder(2, 3, false, Encoder::EncodingType::k4X);//250 ppr *k4x =1000 PPR
		CameraServer::GetInstance()->SetQuality(50);//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");//myRobot.SetExpiration(0.1);
	}
void SetLiftSpeed(float nspeed)
	{
	Lift1.Set(nspeed);
	Lift2.Set(nspeed);
	}
void SetSpeed(float nSpeed)
	{
		SetSpeed(nSpeed, nSpeed);
	}
void SetSpeed(float nLeftSpeed, float nRightSpeed)
	{
		Left1.Set(-nLeftSpeed);
		Left2.Set(-nLeftSpeed);
		Right1.Set(nRightSpeed);
		Right2.Set(nRightSpeed);
	}
void RunLift(int Position)//0->255
	{
		LiftRunning=true;
		float CurrPosition = Map(Pot.GetVoltage(),1.54,0.60,0,255);
		printf("\n CurrPosition:%f,Position:%i", CurrPosition, Position);
		if(CurrPosition<256 && LiftRunning)
		{
			printf("\n CurrPosition:%f", CurrPosition);
			if(CurrPosition>Position)//move down
			{
				while(CurrPosition>Position && LiftRunning)
				{
					printf("\nCurrPosition:%f", CurrPosition);
					CurrPosition = Map(Pot.GetVoltage(),1.54,0.60,0,255);
					SetLiftSpeed(0.5);
				}
				SetLiftSpeed(Stop);
			}
			else if(CurrPosition<Position)//move up
			{
				printf("\n CurrPosition:%f", CurrPosition);
				while(CurrPosition<Position && LiftRunning)
				{
					printf("\n CurrPosition:%f", CurrPosition);
					CurrPosition = Map(Pot.GetVoltage(),1.54,0.60,0,255);
					SetLiftSpeed(-0.7);
				}
				SetLiftSpeed(Stop);
			}
		}
		LiftRunning=false;
	}
void Drive(int distance, float speed)
	{
			float C=2*M_PI*2;
			int Target= (1000/C)*distance;//en/in
			if(distance>0)
			{
				//drive forward until position achieved
				printf("\n target: %i ",Target);
				RightEnc->Reset();
				while(RightEnc->GetRaw()>Target)
				{
					SetSpeed(speed);
				}
				SetSpeed(Stop);
			}
			if(distance<0)
			{
				//drive backward until position achieved
				printf("\n target: %i ",Target);
				RightEnc->Reset();
				while(RightEnc->GetRaw()<Target)
				{
					SetSpeed(-speed);
				}
				SetSpeed(Stop);
			}
	}

void Turn(int Angle)//clockwise is negative
	{
		float circumfrence = 2*(M_PI)*2;
		int ppr = 1000;
		float enc_in=ppr/circumfrence;//result
		float WheelbaseRadius = 25/2;
		float ArcLength=(M_PI/180)*Angle*WheelbaseRadius;//result
		float Tspeed=0.33;
		//float Tbrake =0.04;
		int Ttolerance=15;
		printf("\n ArcLength: %f", ArcLength);
		int Target= abs(round(ArcLength*enc_in));//goal
		PIDLeft.SetTolerance(Ttolerance);
		PIDRight.SetTolerance(Ttolerance);
		PIDLeft.SetOutputRange(-0.35, 0.35);
		PIDRight.SetOutputRange(-0.35, 0.35);
		if(Angle<0)
		{
			LeftEnc->Reset();
			RightEnc->Reset();
			while(LeftEnc->GetRaw()<Target)//turn clockwise
				{
					SetSpeed(Tspeed,-Tspeed);//turn right
					printf("\n Left: %i",LeftEnc->GetRaw());
					printf("\n Right: %i",RightEnc->GetRaw());
				}
			SetSpeed(Stop);
		}
		if(Angle>0)//turn left
		{
			LeftEnc->Reset();
			RightEnc->Reset();
			PIDLeft.SetSetpoint(-Target);
			PIDRight.SetSetpoint(Target);
			PIDLeft.Enable();
			PIDRight.Enable();

			while(true)
			{
			SetSpeed(PIDLeft.Get(), PIDRight.Get());//turn left using pid to correct error
			printf("\n Left: %i",LeftEnc->GetRaw ());
			printf("\n Right: %i",RightEnc->GetRaw());
			Wait(0.005);
			}
			SetSpeed(Stop);
		}
	}


bool ShouldTurn(float J1,float J2)
{
float Maxj =std::max(J1,J2);
float Diff=Maxj-std::min(J1,J2);
float Tolerance = 0.15;
return Diff/Maxj>Tolerance;
}
	void Autonomous()
	{
		 		Turn(90);// tested--Working
//		 		Wait(1.5);
//		 		Turn(-90);//tested--FAIL did not stop
//		 		Wait(1.5);
//		 		Drive(15,0.25);//tested Reverse
//		 		RunLift(127);//tested--Working

	}

	/**
	 *
	 */
	void OperatorControl()
	{
		float lastPValue = 0.00;
		//myRobot.SetSafetyEnabled(true);
		while (IsOperatorControl() && IsEnabled())
		{
			//temp - Test Turn();
			// temp - read POT values
			float curPValue = Map(Pot.GetVoltage(),1.54,0.60,0,255);
			if(curPValue != lastPValue)
			{
				lastPValue = curPValue;
				printf("\n%f", curPValue);
				//printf("\n%f", Pot.GetVoltage());
			}
		//Driver 1
		//Trigger toggle direction--ie reverse motors or joystick input
		//drive with dead zone from Logitech Controller axis

			float J1Y= stick.GetY();
			float J2Y= stick2.GetY();
//			if((abs(J1Y*100)>JMIN*100||abs(J2Y*100)>JMIN*100))
//			{
//				if(ShouldTurn(J1Y, J2Y))
//				{
//					SetSpeed(CalcSpeed(J1Y),CalcSpeed(J2Y));//turn
//				}
//				else
//				{
//					SetSpeed(CalcSpeed(std::max(J1Y, J2Y)));//strait
//				}
//			}
//			else
//			{
//			SetSpeed(Stop);
//			}
			SetSpeed(J1Y/2,J2Y/2);




		//Driver2

			//Toggle smartOverride
			if(stick3.GetRawButton(3)) { // when we press the switch for the first time,
				if(!pressed) { // set as pressed
					if(smartOverride==true) { // when we press it again, it gets turned off
						smartOverride=false;
					} else {
						smartOverride =true;
					}
				}
				pressed = true; // keeping track of pressed allows the button to be
			} else { // held down
			pressed = false;
			}
			//control Lift
		//Position Control?
//
		//CHECK THAT INTELEGENT CONTROLL IS ENABLED
		//CHECK THAT PID OVERIDE IS ENABLED
		if(!smartOverride)//so long as override is toggled off-> PID and Limits
			{
				if((Lim_top.Get()==CLOSED && Lim_base.Get()==CLOSED))//if niether or both switches are pressed
				{
					if(stick3.GetRawButton(1))//if the joystick3 trigger is pressed
					{
						LiftRunning=false;
						printf("/n limit top %i", Lim_top.Get());
						printf("/n limit bottom %i", Lim_base.Get());
						Lift1.Set(-0.7);//up
						Lift2.Set(-0.7);
					}
					else if(stick3.GetRawButton(2))
					{
						LiftRunning=false;
						printf("/n limit top %i", Lim_top.Get());
						printf("/n limit bottom %i", Lim_base.Get());
						Lift1.Set(0.50);//down
						Lift2.Set(0.50);
					}
					else
					{
						if(!LiftRunning)
						{
						Lift1.Set(0.0);
						Lift2.Set(0.0);
						}
					}
				}
				else if(Lim_top.Get()==OPEN && Lim_base.Get()==CLOSED)//if top is pressed->Open
				{
					//printf("\n :::::::::::::::::::POT-top:%f:::::::::::::::::", Pot.GetVoltage());					if(stick3.GetRawButton(2))//only down
					if(stick3.GetRawButton(2)) //only down
					{
						LiftRunning=false;
						printf("/n limit top %i", Lim_top.Get());
						printf("/n limit bottom %i", Lim_base.Get());
						Lift1.Set(0.5);
						Lift2.Set(0.5);
					}
					else
					{

						if(!LiftRunning)
						{
						Lift1.Set(0.0);
						Lift2.Set(0.0);
						}
					}
				}
				else if(Lim_top.Get()==CLOSED && Lim_base.Get()==OPEN)//if bottom is pressed->Open
				{
					//printf("\n :::::::::::::::::::POT-btm:%f:::::::::::::::::", Pot.GetVoltage());
					if(stick3.GetRawButton(1))//only up
					{
						LiftRunning=false;
						printf("/n limit top %i", Lim_top.Get());
						printf("/n limit bottom %i", Lim_base.Get());
						Lift1.Set(-0.7);
						Lift2.Set(-0.7);
					}
					else
					{

						if(!LiftRunning)
						{
						Lift1.Set(0.0);
						Lift2.Set(0.0);
						}
					}
				}
			}
			else if(smartOverride)//toggle to limits only
			{
				if(stick3.GetRawButton(1))//if the joystick3 trigger is pressed
				{
					LiftRunning=false;
					Lift1.Set(-0.7);//up
					Lift2.Set(-0.7);
				}
				else if(stick3.GetRawButton(2))
				{
					LiftRunning=false;
					Lift1.Set(0.50);//down
					Lift2.Set(0.50);
				}
				else
				{
					if(!LiftRunning)
					{
					Lift1.Set(0.0);
					Lift2.Set(0.0);
					}
				}
			}
//		WHAT TO DO IF CONTROLL IS OVERIDED
//		else//toggle to joystick only if the Launchpad switch is in Diable state
//		{
//			if(stick2.GetRawButton(8)==1)
//			{
//				Lift1.Set(0.5);
//				Lift2.Set(0.5);
//			}else if(stick2.GetRawButton(6)==1){
//				Lift1.Set(-0.7);
//				Lift2.Set(-0.7);
//			}else
//			{
//				Lift1.Set(0.0);
//				Lift2.Set(0.0);
//			}
//		}


//***************************************************end of while loop nothing executes after*******************************************************************************************//
		}//while loop
		Wait(0.005);// wait for a motor update time
	}//user method

	/**
	 * Runs during test mode
	 */
	void Test()
	{
	//	m_pdp.GetCurrent(2);
	}

};
START_ROBOT_CLASS(Robot);
