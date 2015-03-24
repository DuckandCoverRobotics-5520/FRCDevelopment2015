#include <UsrMath.h>
#include "WPILib.h"
#include <PID.h>
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
	DigitalInput Lim_base;
	DigitalInput Lim_top;
	DigitalInput Lim_stack;
	Encoder *LeftEnc;
	Encoder *RightEnc;
//	Encoder *LiftEnc;
	AnalogInput Ultra;
	AnalogInput Pot;
	//Compressor mCompressor;
	DoubleSolenoid Pusher;
	const int CurrentLimit=0;//will be defined later
	bool smartOverride = false;
	bool DynamicBraking=false;
	bool LiftRunning = false;
	int LoadPosition=0;
	int StacPusherPosition=0;
	int	PusherPosition;
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
	Lim_base(7),//normally closed
	Lim_top(6),//normally closed
	Lim_stack(8),//normally closed
	Ultra(0),
	Pot(1),//probably not in use...
	Pusher(1,0)
	{
		m_pdp =new PowerDistributionPanel();
		LeftEnc = new Encoder(4, 5, true, Encoder::EncodingType::k4X);//250 ppr *k4x =1000 PPR
		RightEnc =new Encoder(2, 3, false, Encoder::EncodingType::k4X);//250 ppr *k4x =1000 PPR
		Compressor *mCompressor =new Compressor(0);
		//Pusher =new DoubleSolenoid(0,1);
		CameraServer::GetInstance()->SetQuality(50);//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");//myRobot.SetExpiration(0.1);
		mCompressor->SetClosedLoopControl(true);
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
			int LastLeft=0;
			int LastRight=0;
			int DeltaX=0;
			int DeltaY=0;
			float Lspeed=speed;
			float Rspeed=speed;
			float Fastspeed=speed*1.5;
			LeftEnc->Reset();
			RightEnc->Reset();
			int CurrRight=0;
			int CurrLeft=0;
		while((CurrRight<Target && CurrLeft<Target)&&(IsEnabled()&&IsAutonomous()))//may need to or Curr left and right
		{
			CurrRight=RightEnc->Get();
			CurrLeft=LeftEnc->Get();
			int Difference = CurrLeft - CurrRight;

			if(distance>0)//forward
			{
				//is CurrRight>CurrLeft
				//is CurrRight<CurrLeft
				if(CurrRight>CurrLeft)//veering left
				{
				//turn right
					Lspeed=Fastspeed;
					Rspeed=speed;
				}
				if(CurrRight<CurrLeft)//veering Right
				{
					//turn left
					Lspeed=speed;
					Rspeed=Fastspeed;
				}
			}
			if(distance<0)//reverse
			{
				if(CurrRight>CurrLeft)//veering left
				{
				//turn left

				}
				if(CurrRight<CurrLeft)//veering Right
				{
					//turn right

				}
			}
			SetSpeed(Lspeed,Rspeed);
			printf("\n \n LEnc: %i, REnc:%i, Lspeed:%f, Rspeed:%f Target:%i", CurrLeft, CurrRight, Lspeed, Rspeed, Target);
			printf("\n Difference: %i",Difference);
			Wait(0.005);
		}
}

void Turn(int Angle)//clockwise is negative
{
	printf("Angle: %i", Angle);
	float circumfrence = 2*(M_PI)*2;
	int ppr = 1000;
	float enc_in=ppr/circumfrence;//result
	float WheelbaseRadius = 25.25/2;
	float ArcLength=(M_PI/180)*Angle*WheelbaseRadius;//result
	float Tspeed=0.33;
	float Rspeed=0;
	float Lspeed=0;
	//float Tbrake =0.04;
	int Ttolerance=10;
	float ProximityZone = ppr/32;
	printf("\n ArcLength: %f", ArcLength);
	int Target= abs(round(ArcLength*enc_in));//goal
	printf("target: %i", Target);
	if(Angle<0)
	{
		LeftEnc->Reset();
		RightEnc->Reset();
		printf("encoders reset");
		while(RightEnc->GetRaw()!=Target || LeftEnc->GetRaw()!=Target)	//turn left
			{
				int Lenc= LeftEnc->GetRaw();
				int Renc= RightEnc->GetRaw();
				if(std::abs(Target- Renc)>Ttolerance)//if right is not within tolerance
				{
					int RDistance =Target-Renc;
					if(Renc<Target)//if we are not there yet
					{
						printf("\n Renc<Target:%i", RDistance);
						if(RDistance > ProximityZone)//run normally
						{
							Rspeed=-Tspeed;
						}
						if(RDistance < ProximityZone)//slow down were getting close
						{
							Rspeed=-0.15;
						}
					}
					else if(Renc>Target)//if we went too far
					{

						printf("\n Renc>Target:%i", RDistance);
						if(RDistance > ProximityZone)//backup to get closer
						{
							Rspeed=Tspeed;
						}
						if(RDistance < ProximityZone)//slow down were getting close
						{
							Rspeed=0.15;
						}
					}
				}
				else
				{
					printf("\n !Right tolerance achieved!");
					Rspeed=0;
				}
				int PLenc= -1*Lenc;
				 if(std::abs(Target-PLenc)>Ttolerance)//if left is not within tolerance
				{
					if(PLenc< Target)//if we are not there yet
					{
						int Distance = Target - PLenc;
						printf("\n PLenc< Target:%i", Distance);
						if(Distance > ProximityZone)
						{
							Lspeed=Tspeed;
						}
						if(Distance < ProximityZone)
						{
							Lspeed=0.15;
						}
					}
					else if(PLenc> Target)//if we went too far
					{
						int Distance = Target - PLenc;
						printf("\n PLenc< Target:%i", Distance);
						if(Distance > ProximityZone)
						{
							Lspeed=-Tspeed;
						}
						if(Distance < ProximityZone)
						{
							Lspeed=-0.15;
						}
					}
				}
				else
				{
					Lspeed=0;
					printf("\n !Left tolerance achieved!");
				}
					printf("\n SetSpeed( %f , %f ) \n",Lspeed,Rspeed);
					SetSpeed(Lspeed,Rspeed);//turn left
					if(Lspeed==0 && Rspeed==0)
					{
						printf("::::::::::::::Turn( %i ) -> Achieved:::::::::::::::::::", Angle);
						break;
					}
					//printf("\n Left: %i",LeftEnc->GetRaw ());
					//printf("\n Right: %i",RightEnc->GetRaw());
			}//end while(condition)
		SetSpeed(Stop);
	}//end Angle (negative) right turn v2 working.


	if(Angle>0)
			{
				LeftEnc->Reset();
				RightEnc->Reset();
				while(RightEnc->GetRaw()!=Target || LeftEnc->GetRaw()!=Target)	//turn left
					{
						int Lenc= LeftEnc->GetRaw();
						int Renc= RightEnc->GetRaw();
						if(std::abs(Target- Renc)>Ttolerance)//if right is not within tolerance
						{
							if(Renc<Target)//if we are not there yet
							{
								int Distance =Target-Renc;
								printf("\n Renc<Target:%i", Distance);
								if(Distance > ProximityZone)//run normally
								{
									Rspeed=-Tspeed;
								}
								if(Distance < ProximityZone)//slow down were getting close
								{
									Rspeed=-0.15;
								}
							}
							else if(Renc>Target)//if we went too far
							{
								int Distance =Target-Renc;
								printf("\n Renc>Target:%i", Distance);
								if(Distance > ProximityZone)//backup to get closer
								{
									Rspeed=Tspeed;
								}
								if(Distance < ProximityZone)//slow down were getting close
								{
									Rspeed=0.15;
								}
							}
						}
						else
						{
							printf("\n !Right tolerance achieved!");
							Rspeed=0;
						}
						int PLenc= -1*Lenc;
						 if(std::abs(Target-PLenc)>Ttolerance)//if left is not within tolerance
						{
							if(PLenc< Target)//if we are not there yet
							{
								int Distance = Target - PLenc;
								printf("\n PLenc< Target:%i", Distance);
								if(Distance > ProximityZone)
								{
									Lspeed=Tspeed;
								}
								if(Distance < ProximityZone)
								{
									Lspeed=0.15;
								}
							}
							else if(PLenc> Target)//if we went too far
							{
								int Distance = Target - PLenc;
								printf("\n PLenc< Target:%i", Distance);
								if(Distance > ProximityZone)
								{
									Lspeed=-Tspeed;
								}
								if(Distance < ProximityZone)
								{
									Lspeed=-0.15;
								}
							}
						}
						else
						{
							Lspeed=0;
							printf("\n !Left tolerance achieved!");
						}
							printf("\n SetSpeed( %f , %f ) \n",Lspeed,Rspeed);
							SetSpeed(Lspeed,Rspeed);//turn left
							if(Lspeed==0 && Rspeed==0)
							{
								printf("Turn( %i ) -> Achieved", Angle);
								break;
							}
							//printf("\n Left: %i",LeftEnc->GetRaw ());
							//printf("\n Right: %i",RightEnc->GetRaw());
					}//end while(condition)
				SetSpeed(Stop);
			}//end Angle (positive) left turn v1
}//end Void Turn
void TestSimMtr()
{
	float lastLValue = 0.00;
	float lastRValue = 0.00;
	while(IsEnabled()&&IsTest())
	{
	SetSpeed(1,1);
		int curLValue = LeftEnc->GetRaw();
		if(curLValue != lastLValue)
		{
			lastLValue = curLValue;
			printf("\n %i", curLValue);
			//printf("\n%f", Pot.GetVoltage());
		}
		int curRValue = LeftEnc->GetRaw();
		if(curRValue != lastRValue)
		{
			lastRValue = curRValue;
			printf(" %i", curRValue);
			//printf("\n%f", Pot.GetVoltage());
		}
//	Wait(0.005);
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
//Post Bag Day
//				Turn(90);// tested--Working.
//				Wait(1.5);
//		 		Turn(-90);//tested--Working.
				Drive(1000,0.3);
//Pre-Bag-Day
	//				Turn(90);// tested--fail not stop
	//				Wait(1.5);
	//		 		Turn(-90);//tested--Working
	//		 		Wait(1.5);
	//		 		Drive(15,0.25);//tested Reverse
	//		 		RunLift(127);//tested--Working
	//  			TestSimMTR();

	}

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
			//pnumatic pusher
			if(stick2.GetRawButton(1))
			{
				Pusher.Set(DoubleSolenoid::kForward);//forward
				PusherPosition=1;
			}
			else if (PusherPosition==1 && !(stick2.GetRawButton(1)))
			{
				Pusher.Set(DoubleSolenoid::kReverse);//reverse
				PusherPosition=0;//exception to make this occurrence possible only once after every actuation
				Wait(0.05);//give the valve time to open--(0.005-way too fast), (0.01-too fast), (0.05-good timing, occasionally misses +buggy), may need to add more time
			}else
			{
				Pusher.Set(DoubleSolenoid::kOff);//off
				PusherPosition=-1;//for diagnostics and consistency
			}



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
				else if(Lim_top.Get()==OPEN && Lim_base.Get()==CLOSED)//if top is pressed->Open
				{
					//printf("\n :::::::::::::::::::POT-top:%f:::::::::::::::::", Pot.GetVoltage());					if(stick3.GetRawButton(2))//only down
					if(stick3.GetRawButton(2)) //only down
					{
						LiftRunning=false;
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
		//WHAT TO DO IF CONTROLL IS OVERIDED
			else if(smartOverride)//toggle to no limits
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

	void Test()
	{
		LeftEnc->Reset();
		while(IsTest()&&IsEnabled())
		{
		printf("\n LeftEnc:%i",LeftEnc->GetRaw());
		Left1.Set(PID(1000,LeftEnc->GetRaw()));
		Wait(0.005);
		}
	}

};
START_ROBOT_CLASS(Robot);
