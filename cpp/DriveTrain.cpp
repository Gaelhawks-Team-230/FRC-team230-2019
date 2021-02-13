/*
DriveTrain.cpp

Created by: Suhaas Nadella on 1/14/2019

____
|DD|____T_
|_ |_____|<
  @-@-@-oo\

*/


#include "TalonXX_main.h"
#include <frc/WPILib.h>
#include "ctre/Phoenix.h"
#include "DriveTrain.h"


//constructor
DriveTrain::DriveTrain(TalonXX* pRobot)
{
    frontLeftMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX (CAN_FRONT_LEFT);
    frontRightMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX (CAN_FRONT_RIGHT);
    backLeftMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX (CAN_BACK_LEFT);
    backRightMotor = new ctre::phoenix::motorcontrol::can::WPI_TalonSRX (CAN_BACK_RIGHT);


    frontLeftMotorCmd = 0.0;
    frontRightMotorCmd = 0.0;
    backLeftMotorCmd = 0.0;
    backRightMotorCmd = 0.0;
    
    limitDriveCmd = 0.0;
    driveDelta = 2.0;

    mainRobot = pRobot;

#ifdef USE_GYRO
	gyro = new frc::ADXRS450_Gyro(frc::SPI::kOnboardCS0);
	gyro->Calibrate();
	gyroVel = 0;
	gyroAngle = 0;
	gyroErr = 0;
	gyroErrInt = 0;
	gyroOn = true;
    gyroReading = 0.0;
    oldGyroReading = 0.0;
#else
	gyroOn = false;
#endif
	LocalReset();
}

void DriveTrain::DriveControl(float driveCmd, float strafeCmd, float rotateCmd, float forcedDrive, float forcedStrafe, float forcedRotate, bool isJoystickCmd, bool isAuto, bool isPaused)
{

    if(isPaused)
    {
        //save all values but stop motors
        frontLeftMotor->Set(0.0);
        frontRightMotor->Set(0.0);
        backLeftMotor->Set(0.0);
        backRightMotor->Set(0.0);
        return;
    }
    
    double modifiedRotate;
    if(isJoystickCmd)
    {
        JoystickShaping(&driveCmd, &strafeCmd, &rotateCmd, isAuto);
        double strafeMultiply;
        strafeMultiply = (1 + mainRobot->joystick->GetRawAxis(3))/2;
        strafeCmd = strafeCmd + forcedStrafe - strafeMultiply*rotateCmd;
    }
   
    
    driveCmd += forcedDrive; 
   // double rotateMultiplier;
   // rotateMultiplier = 0.25 + 0.75*((1 + mainRobot->joystick->GetRawAxis(4))/2);
   // rotateCmd *= rotateMultiplier;
    rotateCmd += forcedRotate;
   // rotateCmd *= 0.8;                   //TESTING!!!
  //  double strafeMultiply;
  //  strafeMultiply = (1 + mainRobot->joystick->GetRawAxis(3))/2;
  //  strafeCmd = strafeCmd + forcedStrafe - strafeMultiply*rotateCmd;
    strafeCmd += forcedStrafe;
  

    double tempLimit = 1 - (mainRobot->lift->GetHeight() - 18)/90;    //(72.0 - mainRobot->lift->GetHeight())/72.0*0.5 + 0.4;
    limitDriveCmd = TalonXX::Limit(-tempLimit, tempLimit, driveCmd);
    
    //without modified rotate
    #ifdef USE_GYRO
        double teleOpMaxRotate;
        
        teleOpMaxRotate =  MAX_ROTATE * RAD_TO_DEG; 
        if(gyroOn)
        {
            if(isAuto)
            {
                modifiedRotate = GyroControl(rotateCmd);
            }
            else    
            {
                modifiedRotate = GyroControl(rotateCmd * teleOpMaxRotate);
            }
        }
        else
        {
            //printf("GyroOff?\n");
            //teleOpMaxRotate = 0.6; //1.0
            modifiedRotate = 0.0035 * teleOpMaxRotate * rotateCmd;
        }
    #else
        modifiedRotate = rotateCmd;
    #endif

    frontLeftMotorCmd = Limit(limitDriveCmd + strafeCmd + modifiedRotate);
	frontLeftMotor->Set(frontLeftMotorCmd);

	frontRightMotorCmd = Limit(limitDriveCmd - strafeCmd - modifiedRotate);
	frontRightMotor->Set(-1.0*frontRightMotorCmd); //*-1 is for opposite side of robot

	backLeftMotorCmd = Limit(limitDriveCmd - strafeCmd + modifiedRotate);
	backLeftMotor->Set(backLeftMotorCmd);

	backRightMotorCmd = Limit(limitDriveCmd + strafeCmd - modifiedRotate);
	backRightMotor->Set(-1.0*backRightMotorCmd);
}

//includes 2 parts
void DriveTrain::JoystickShaping(float *driveCmd, float *strafeCmd, float *rotateCmd, bool isAuto)
{
    if(!isAuto)
    {
        if(fabs(*driveCmd) <= DRIVE_DEADBAND)
        {
            *driveCmd = 0.0;
        }
        else if(*driveCmd > 0.0)
        {
            *driveCmd = (*driveCmd - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
        }
        else
        {
            *driveCmd = (*driveCmd + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
        }
    

        if(fabs(*rotateCmd) <= DRIVE_DEADBAND)
        {
            *rotateCmd = 0.0;
        }
        else if(*rotateCmd > 0.0)
        {
            *rotateCmd = (*rotateCmd - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
        }
        else
        {
            *rotateCmd = (*rotateCmd + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
        }


        if(fabs(*strafeCmd) <= DRIVE_DEADBAND)
        {
            *strafeCmd = 0.0;
        }
        else if(*strafeCmd > 0.0)
        {
            *strafeCmd = (*strafeCmd - DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
        }
        else
        {
            *strafeCmd = (*strafeCmd + DRIVE_DEADBAND)/(1.0 - DRIVE_DEADBAND);
        }
    }

    *strafeCmd = *strafeCmd * (1.0 + 0.5 * SIDE_M_CMD * (fabs(*strafeCmd) - 1.0));
	*rotateCmd = *rotateCmd * (1.0 + 0.5 * M_CMD * (fabs(*rotateCmd) - 1.0));
	*driveCmd = *driveCmd * (1.0 + 0.5 * M_CMD * (fabs(*driveCmd) - 1.0));
}


void DriveTrain::GyroOff()
{
    gyroOn = false;
    #ifdef USE_GYRO
        gyroErrInt = 0.0;
    #endif
}

//tells if gyro is on
void DriveTrain::GyroOn()
{
    #ifdef USE_GYRO
        gyroOn = true;
    #else
        gyroOn = false;
    #endif
}

double DriveTrain::GyroControl(double motorCmd)
{
#ifdef USE_GYRO
	//gyroVel = GetGyroVelocity();
	gyroErr = motorCmd - gyroVel;
	gyroErrInt = gyroErrInt + (gyroErr * LOOPTIME);
	float newCmd = (K_GYRO / K_ROBOT) * (TAU_ROBOT * gyroErr + gyroErrInt);

	return newCmd;
#else

	return motorCmd;
#endif
}

double DriveTrain::GetGyroAngle()
{
    double gyroA;
    #ifdef USE_GYRO
        gyroA = gyro->GetAngle();
        return gyroA;
    #else
        return 0.0;
    #endif
}

double DriveTrain::GetGyroVelocity()
{
    double gyroV;
    #ifdef USE_GYRO
        //gyroV = gyro->GetRate();
        gyroReading = gyro->GetAngle();
        gyroV = (gyroReading - oldGyroReading)/LOOPTIME;
        oldGyroReading = gyroReading;
        //gyroV = TalonXX::Limit(-300.0, 300.0, gyroV);
        return gyroV;
    #endif
}

//checks if the motor cmds are over 0.1
bool DriveTrain::IsRobotMoving()
{
    if(fabs(frontLeftMotorCmd) > MIN_MOVING_CMD && fabs(frontRightMotorCmd) > MIN_MOVING_CMD && fabs(backLeftMotorCmd) > MIN_MOVING_CMD && fabs(backRightMotorCmd) > MIN_MOVING_CMD)
		return true;
	else
		return false;
}

//limits the cmds on controller
double DriveTrain::Limit(double num)
{
	if(num > MAX_CMD)
		return MAX_CMD;
	else if(num < MIN_CMD)
		return MIN_CMD;
	else
		return num;
}

void DriveTrain::LocalReset()
{
    //Reset all variables
    frontRightMotorCmd = 0.0;
    frontLeftMotorCmd = 0.0;
    backRightMotorCmd = 0.0;
    backLeftMotorCmd = 0.0;
   // gyroOn = false;


    #ifdef USE_GYRO
        gyroOn = true;
        gyro->Reset();
        gyroReading = gyro->GetAngle();
        oldGyroReading = gyroReading;
        gyroErrInt = 0.0;
    #else
        gyroOn = false;
    #endif
}

void DriveTrain::StartingConfig()
{

}

void DriveTrain::StopAll()
{
    LocalReset();
}

void DriveTrain::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Front Left", frontLeftMotorCmd);
    frc::SmartDashboard::PutNumber("Front Right", frontRightMotorCmd);
    frc::SmartDashboard::PutNumber("Back Left", backLeftMotorCmd);
    frc::SmartDashboard::PutNumber("Back Right", backRightMotorCmd);
    frc::SmartDashboard::PutNumber("Gyro rate", gyroVel);
    frc::SmartDashboard::PutNumber("Gyro angle", gyroAngle);
    frc::SmartDashboard::PutBoolean("GyroOn ", gyroOn);
}


void DriveTrain::Service()
{
    gyroAngle = GetGyroAngle();
    gyroVel = GetGyroVelocity();
}
