/*
DriveTrain.h

Created by: Suhaas Nadella on 1/14/1894

____
|DD|____T_
|_ |_____|<
  @-@-@-oo\
  
*/

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_


#include "ctre/Phoenix.h"
#include "Common.h"
#include "TalonXX_main.h"
#include <frc/WPILib.h>
//#include <frc/TalonSRX.h>

class TalonXX;

//constants from 2018
#define K_GYRO                 (20.0)
#define K_ROBOT                (610)
#define TAU_ROBOT              (0.15)

#define SIDE_M_CMD             (0.0)
#define M_CMD                  (0.7)

#define MAX_ROTATE             (2.4)//(3.0)

#define GAMEPAD_M_CMD          (0.7)
#define MAX_CMD                (1.0)
#define MIN_CMD                (-1.0)

#define MIN_MOVING_CMD    (0.1)
//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)

class DriveTrain
{
    private:
        ctre::phoenix::motorcontrol::can::WPI_TalonSRX  *frontLeftMotor;
	    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  *frontRightMotor;
	    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  *backLeftMotor;
	    ctre::phoenix::motorcontrol::can::WPI_TalonSRX  *backRightMotor;
    

        TalonXX *mainRobot;

#ifdef USE_GYRO
        frc::ADXRS450_Gyro *gyro;
        double gyroVel;
        double gyroAngle;
        double gyroErr;
        double gyroErrInt;
        double limitDriveCmd;
        double driveDelta;
#endif
        bool gyroOn;

        double frontLeftMotorCmd;
        double frontRightMotorCmd;
        double backLeftMotorCmd;
        double backRightMotorCmd;
        double gyroReading;
        double oldGyroReading;


        double Limit(double num);
        //declare member variables
        //example: float height;

    public:
        DriveTrain(TalonXX* pRobot);
        void DriveControl(float driveCmd, float strafeCmd, float rotateCmd, float, float, float, bool isJoystickCmd, bool isAuto, bool isPaused);
        bool IsRobotMoving(void);
        void GyroOn(void);
        void GyroOff(void);
        double GyroControl(double motorCmd);
        double GetGyroVelocity(void);
        double GetGyroAngle(void);
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);
        void JoystickShaping(float *driveCmd, float *strafeCmd, float *rotateCmd, bool isAuto);
};
#endif /*DRIVETRAIN_H_*/
