/*
DriveDistance.cpp

Created by Suhaas (Feb 2, 2019)


*/

#include "Common.h"
#include "DriveDistance.h"
#include "TalonXX_main.h"
#include <frc/WPILib.h>
#include "ctre/Phoenix.h"


DriveDistance::DriveDistance(TalonXX* pRobot)
{
    mainRobot = pRobot;
    distancePerPulse = (WHEEL_CIRCUMFERENCE/ENCODER_PULSE_COUNT);
	encoderFL = new frc::Encoder(FRONT_LEFT_ENCODER_ONE, FRONT_LEFT_ENCODER_TWO, false);
	encoderFR = new frc::Encoder(FRONT_RIGHT_ENCODER_ONE, FRONT_RIGHT_ENCODER_TWO, false);
	encoderBL = new frc::Encoder(BACK_LEFT_ENCODER_ONE, BACK_LEFT_ENCODER_TWO, false);
	encoderBR = new frc::Encoder(BACK_RIGHT_ENCODER_ONE, BACK_RIGHT_ENCODER_TWO, false);

	encoderFL->SetDistancePerPulse(-1.0 * distancePerPulse);
	encoderFR->SetDistancePerPulse(distancePerPulse);
	encoderBL->SetDistancePerPulse(-1.0 * distancePerPulse);
	encoderBR->SetDistancePerPulse(distancePerPulse);

    LocalReset();
}

void DriveDistance::LocalReset()
{
    ResetEncoders();
    //Reset all variables
     fwdDis = 0.0;
     leftDis = 0.0;
     rightDis = 0.0;
     diagDis = 0.0;
     LsideDis = 0.0;
     RsideDis= 0.0;
     averageDis = 0.0;
     frontDis = 0.0;
     backDis = 0.0;

}

void DriveDistance::ResetEncoders()
{
    encoderFL->Reset();
	encoderFR->Reset();
	encoderBL->Reset();
	encoderBR->Reset();
}

void DriveDistance::StartingConfig()
{

}

void DriveDistance::StopAll()
{
    ResetEncoders();
}

void DriveDistance::UpdateDash()
{
	frc::SmartDashboard::PutNumber("Forwards-Backwards Distance", GetForwardsDistance());
	//frc::SmartDashboard::PutNumber("Strafe Distance", GetStrafingDistance());
	frc::SmartDashboard::PutBoolean("Drive Encoder Status", EncoderStatus());
	frc::SmartDashboard::PutNumber("FL Encoder", encoderFL->GetDistance());
	frc::SmartDashboard::PutNumber("FR Encoder", encoderFR->GetDistance());
	frc::SmartDashboard::PutNumber("BL Encoder", encoderBL->GetDistance());
	frc::SmartDashboard::PutNumber("BR Encoder", encoderBR->GetDistance());
}

bool DriveDistance::CheckEncoders(int encoderCase1, int encoderCase2)
{

	if (abs(encoderCase1 - encoderCase2) >= ENCODER_PULSE_CHECK)
		return false;
	else
		return true;
}

double DriveDistance::GetRightDistance()
{
	rightDis = (encoderFR->GetDistance() + encoderBR->GetDistance())/2;
	return rightDis;
}
bool DriveDistance::EncoderStatus()
{
	return true;

	if(CheckEncoders(frontDis, backDis) == false)
		return false;
}

double DriveDistance::GetLeftDistance()
{
	leftDis = (encoderFL->GetDistance() + encoderBL->GetDistance())/2;
	return leftDis;
}

double DriveDistance::GetForwardsDistance()
{
	frontDis = (encoderFR->Get() + -1 * encoderFL->Get())/2;
	backDis = (encoderBR->Get() + -1 * encoderBL->Get())/2;

	fwdDis = (frontDis + backDis)/2;

	if (CheckEncoders(frontDis, backDis))
		return (fwdDis * distancePerPulse);
	else if (frontDis > backDis)
		return (frontDis * distancePerPulse);
	else
		return (backDis * distancePerPulse);
}

//gets distance traveled when strafing using check encoder function. combines left and right negative means left, positive means right
double DriveDistance::GetStrafingDistance()
{
	double RsideDis1;
	double RsideDis2;
	double Rvalue;

	RsideDis1 = (encoderFL->Get() - encoderBL->Get());
	RsideDis2 = (encoderBR->Get() - encoderFR->Get());


	RsideDis = (RsideDis1 + RsideDis2)/2;


	double LsideDis1;
	double LsideDis2;
	double Lvalue;

	LsideDis1 = (encoderFR->Get() - encoderBR->Get());
	LsideDis2 = (encoderBL->Get() - encoderFL->Get());

	LsideDis = (LsideDis1 + LsideDis2)/2;

	if (CheckEncoders(RsideDis1, RsideDis2))
		Rvalue = (RsideDis * distancePerPulse);
	else if (RsideDis1 > RsideDis2)
		Rvalue = (RsideDis1 * distancePerPulse);
	else
		Rvalue = (RsideDis2 * distancePerPulse);

	if (CheckEncoders(LsideDis1, LsideDis2))
		Lvalue = (LsideDis * distancePerPulse);
	else if (LsideDis1 > LsideDis2)
		Lvalue = (LsideDis1 * distancePerPulse);
	else
		Lvalue = (LsideDis2 * distancePerPulse);

	return (Lvalue + Rvalue);
	return Lvalue;
	return Rvalue;
}

//Called every loop
void DriveDistance::Service()
{
    
}
