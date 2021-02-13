//#include <frc/WPILib.h>
#include <frc/VictorSP.h>
#include "Common.h"
#include "CargoClaw.h"

CargoClaw::CargoClaw(TalonXX* pRobot)
{
    mainRobot = pRobot;
	wheels = new frc::VictorSP(PWM_CLAW_WHEELS);    
	cargoSensor = new frc::DigitalInput(DIGIN_CARGO_SENSOR);
    tilt = new frc::VictorSP(PWM_CLAW_TILT);
    #ifndef PRACTICE_BOT
    tiltMultiplier = -1;
    #else
    tiltMultiplier = 1;
    #endif
    angleEncoder = new frc::Encoder(CLAW_ANGLE_ENCODER_CHANNEL_A, CLAW_ANGLE_ENCODER_CHANNEL_B);
    angleEncoder->SetDistancePerPulse(TILT_DISTANCE_PER_PULSE);
    angleEncoder->Reset();
    LocalReset();
    isManualOverride = false;
    isCalibrating = true;
    encoderReset = true;
    calilbrationCount = 0;
    tiltAdjustZero = 0.0;
    tiltAdjust = 0.0;
    currentTiltAdjust = 0.0;
}

void CargoClaw::LocalReset()
{
    isGrabbingCargo = false;
    isGrabbingCargoTilt = false;
    isReleasingCargo = false;
    isManualOverride = false;
    wheelMotor = 0.0;
    tiltMotor = 0.0;
    manualRotateCmd = 0.0;

    currentAngle = GetAngle();
    goalAngle = currentAngle;
    oldPcmd = currentAngle;
    isManuallyMoving = false;
    isAtGoal = false;
    posCmd = currentAngle;
    posError = 0.0;
    velocity = 0.0;
    oldAngle = currentAngle;
    velCmd = 0.0;
    atGoalCount = 0;
    velError = 0.0;
    velErrorInt = 0.0;
    oldVelocity = 0.0;

    tiltShift = 0.0;

    releaseStage = 0;
    releaseLoopCount = 0;
    testLoopCount = 0;

    hasCargo = HaveCargo();

    wheels -> Set(wheelMotor);
    tilt -> Set(tiltMotor);
}

void CargoClaw::StartingConfig()
{
    StopWheelSpin();
    goalAngle = TOP_STOP;
}

void CargoClaw::StopAll()
{
    LocalReset();
}

void CargoClaw::StopWheelSpin()
{
    wheelMotor = 0.0;
    isReleasingCargo = false;
    isGrabbingCargo = false;
}

void CargoClaw::IntakeCargo()
{
    wheelMotor = INTAKE_CARGO_CMD;
    isReleasingCargo = false;
    isGrabbingCargo = true;
    isGrabbingCargoTilt = true;
#ifdef USE_VACUUM
    mainRobot->hatchPanel->ReleaseVacuum();
#endif
}

bool CargoClaw::ReturnIsGrabbingCargoTilt()
{
    return isGrabbingCargoTilt;
}

void CargoClaw::EjectCargo()
{
    wheelMotor = OUTPUT_CARGO_CMD;
    isGrabbingCargo = false;
    isReleasingCargo = true;
    isGrabbingCargoTilt = false;
}

double CargoClaw::ReturnGoalAngle()
{
    return goalAngle;
}

void CargoClaw::TurnOffIsGrabbingCargoTilt()
{
    isGrabbingCargoTilt = false;
}

void CargoClaw::SetGoalAngle(double inputAngle)
{
    isManuallyMoving = false;
    goalAngle = inputAngle;
}

bool CargoClaw::AtMaxAngle()
{
    if (fabs (currentAngle-TOP_STOP) <= TILT_ERROR)
    {
        return true;
    }
    return false;
}

bool CargoClaw::AtMinAngle()
{
    if (fabs (currentAngle-BOTTOM_STOP) <= TILT_ERROR)
    {
        return true;
    }
    return false;
}

void CargoClaw::ManualRotate(double angleCmd)
{
    if (fabs(angleCmd) >= MANUAL_ANGLE_DEADBAND)
    {
        isGrabbingCargoTilt = false;

        if (angleCmd > 0.0)
        {
            if(!isManuallyMoving)
            {
                goalAngle = currentAngle;
                isManuallyMoving = true;
            }
            else
            {
                goalAngle = goalAngle + (MANUAL_UP_INCREMENT * angleCmd);
            }
        }
        else
        {
            if(!isManuallyMoving)
            {
                goalAngle = currentAngle;
                isManuallyMoving = true;
            }
            else
            {
                goalAngle = goalAngle - (MANUAL_DOWN_INCREMENT * angleCmd);
            }
        }
    }
    else
    {
        return;
    }

    goalAngle = TalonXX::Limit(BOTTOM_STOP, TOP_STOP, goalAngle);
}

void CargoClaw::OverrideManualRotate(double overrideCmd)
{
    if (fabs (overrideCmd) > MANUAL_ANGLE_DEADBAND)
    {
        tiltMotor = overrideCmd * TILT_OVERRIDE_CONSTANT;
    }
    else
    {
        tiltMotor = 0.0;
    }
}

void CargoClaw::OverrideStop()
{
    tiltMotor = 0.0;
}

void CargoClaw::ManualOverrideOn()
{
    isManualOverride = true;
}

void CargoClaw::ManualOverrideOff()
{
    isManualOverride = false;
    LocalReset();
}

bool CargoClaw::isOverrideOn()
{
    return isManualOverride;
}

void CargoClaw::StopClawTilt()
{
    goalAngle = currentAngle;
}

double CargoClaw::GetAngle()
{
    double encoderReading;
    encoderReading = angleEncoder->GetDistance() - tiltAdjust;// + CALIBRATION_DELTA;
    return encoderReading; //+ tiltShift
}

/*void CargoClaw::TiltAdjust(double slideReading)
{
    if(slideReading > 0.0)
    {
        tiltShift = slideReading * TILT_SLIDER_CONSTANT;
    }
    else
    {
        tiltShift =  0.0;
    }
    
}*/

void CargoClaw::ReCalibrate()
{
    if(!isCalibrating)
    {
        isManualOverride = false;
        isCalibrating = true;
        encoderReset = true;
        calilbrationCount = 0;
       
    }
}

double CargoClaw::AngleControlSystem()
{
    if(!isCalibrating)
    {
        goalAngle = TalonXX::Limit(BOTTOM_STOP, TOP_STOP, goalAngle);
    }
    if(calilbrationCount > CALIBRATION_CMD_COUNT)
    {
        if((int)(currentAngle * 10000) == (int) (oldAngle * 10000))
        {
            isCalibrating = false;
            if(encoderReset)
            {
                angleEncoder->Reset();
                tiltAdjustZero = currentTiltAdjust;
                tiltAdjust = 0.0;
                encoderReset = false;
                LocalReset();                //printf("Tilt Encoder Reset");
            }
            //printf("Tilt Done Calibating \n");
        }
    }
    calilbrationCount++;
    
    
    posCmd = posCmd + TalonXX::Limit(DOWN_INCREMENT, UP_INCREMENT, goalAngle - posCmd);
    
  //  posError = posCmd - currentAngle;
    posError = posCmd + TILT_KFF*((posCmd - oldPcmd)/LOOPTIME) - currentAngle;
    velocity = (currentAngle-oldAngle)/LOOPTIME;
    velocity = TalonXX::Limit(TILT_VEL_DOWN_LIMIT, TILT_VEL_UP_LIMIT, velocity);
    
    if(isCalibrating)
    {
        velCmd = -20.0;
    }
    else
    {
        if(fabs(goalAngle-currentAngle)<=TILT_ERROR)
        {
            //velCmd = 0.0;
            isAtGoal = true;
        }
        else 
        {
           // velCmd = (TILT_KP * (posError)) + TILT_KFF*((posCmd - oldPcmd)/LOOPTIME);
            isAtGoal = false;
        }
        velCmd = TILT_KP*posError;
    }
    
    velError = velCmd - velocity;
    velErrorInt = velErrorInt + (velError * LOOPTIME);
    velErrorInt = TalonXX::Limit(TILT_INT_LOW_LIMIT, TILT_INT_HIGH_LIMIT, velErrorInt);

  /*  double velErrIntMin;
    velErrIntMin = TILT_INT_LOW_LIMIT*TalonXX::Limit(0.0,1.0, currentAngle * 0.25);
    velErrorInt = TalonXX::Limit(velErrIntMin, TILT_INT_HIGH_LIMIT, velErrorInt);*/

    tiltMotor = TILT_KBANDWIDTH/TILT_K*(TILT_TAU*velError+velErrorInt);
    if(isCalibrating)
    {
        tiltMotor = TalonXX::Limit(-0.5, 0.5, tiltMotor);
    }
    oldAngle = currentAngle;
    oldVelocity = velocity;
    oldPcmd = posCmd;
}

bool CargoClaw::HaveCargo()
{
    if (cargoSensor->Get())
	{
		hasCargo = false;
	}
	else
	{
		hasCargo = true;
	}
    return hasCargo;
}

void CargoClaw::ResetTiltAdjust(double tiltAdjustZero_In)
{
    tiltAdjustZero = tiltAdjustZero_In;
}

void CargoClaw::DialTiltAdjust(double tiltAdjust_In)
{
    currentTiltAdjust = tiltAdjust_In;
    tiltAdjust = TILT_ADJUST_CONSTANT * (currentTiltAdjust - tiltAdjustZero);
}

bool CargoClaw::IsDialAdjusted()
{
    if(fabs(tiltAdjust/TILT_ADJUST_CONSTANT) > 0.1)
    {
        return false;
    }
    return true;
}

void CargoClaw::WheelService()
{
    hasCargo = HaveCargo();
    if (isGrabbingCargo)
    {
        if (hasCargo)//sensor info
        {
            StopWheelSpin();
        }

        else
        {   
            IntakeCargo();
        }
    }
    
    
}

void CargoClaw::AngleService()
{
    currentAngle = GetAngle();
    goalAngle = TalonXX::Limit(BOTTOM_STOP, TOP_STOP, goalAngle);
    
    if (!isManualOverride)
    {
        AngleControlSystem();
       // printf("%f %f %f %f %f ", tiltMotor, posCmd, currentAngle, velCmd, velocity);

    }

    tiltMotor = tiltMotor * tiltMultiplier;

    //if(isGrabbingCargoTilt)
    if(hasCargo)
    {
        CargoAutoAngle();
    }
    testLoopCount++;
}


//Called every loop
void CargoClaw::Service()
{
    if (mainRobot->isPaused && !isCalibrating)
    {
        wheels->Set(0.0);
        tilt->Set(0.0);
        return;
    }
    //printf("tiltAdjust %f", tiltAdjust);
    //code will only do this if the robot is not paused
    WheelService();
    AngleService();
    wheels->Set(wheelMotor);
    tilt->Set(tiltMotor);
}

void CargoClaw::UpdateDash()
{
    //frc::SmartDashboard::PutNumber("Wheel Motor Speed: ", wheelMotor);
    frc::SmartDashboard::PutNumber("Tilt Motor Speed: ", tiltMotor);
    frc::SmartDashboard::PutNumber("Tilt Goal Angle: ", goalAngle);
    frc::SmartDashboard::PutBoolean("Has Cargo: ", hasCargo);
    //frc::SmartDashboard::PutBoolean("Grabbing Cargo: ", isGrabbingCargo);
    //frc::SmartDashboard::PutBoolean("Releasing Cargo: ", isReleasingCargo);
    frc::SmartDashboard::PutBoolean("TiltIsOverride: ", isManualOverride);
  //  frc::SmartDashboard::PutBoolean("Tilt Manual Move: ", isManuallyMoving);
 //   frc::SmartDashboard::PutNumber("TiltCurrentAngle: ", GetAngle());
        frc::SmartDashboard::PutNumber("TiltCurrentAngle: ", currentAngle);
        frc::SmartDashboard::PutBoolean("Tilt Calibrated: ", !isCalibrating);
      // frc::SmartDashboard::PutBoolean("Tilt Limit Override: ", tiltLimitOverride);
        frc::SmartDashboard::PutNumber("TiltAdjust", tiltAdjust);
        frc::SmartDashboard::PutBoolean("IsTiltAdjusted", IsDialAdjusted());
    //frc::SmartDashboard::PutBoolean("Tilt At Max", AtMaxAngle());
   // frc::SmartDashboard::PutBoolean("Tilt At Bottom", AtMinAngle());
   // frc::SmartDashboard::PutBoolean("Tilt At Goal", isAtGoal);

}

bool CargoClaw::AtGoalAngle()
{
    if(fabs(goalAngle - currentAngle) <= AT_GOAL_ALLOWANCE)
	{
		isAtGoal = true;
	}
	else
	{
		isAtGoal = false;
	}
	return isAtGoal;
}

void CargoClaw::CargoAutoAngle()
{
    if(mainRobot->lift->GetHeight() > CARGO_TILT_BOTTOM_LIMIT)
    {
        goalAngle = TalonXX::Min(TOP_STOP, TILT_ANGLE_CONSTANT_CARGO*(mainRobot->lift->GetHeight() - CARGO_TILT_BOTTOM_LIMIT) + TOP_STOP);
    }
}

void CargoClaw::FeederStationRelease()
{
    switch(releaseStage)
    {
        case 0:
            SetGoalAngle(FULL_BACK_POS);
            releaseStage++;
            break;

        case 1:
            if(AtGoalAngle())
            {
                releaseStage++;
            }
            break;

        case 2:
            if(releaseLoopCount <= DRIVE_BACK_TIME)
            {
                mainRobot->driveCmd = -0.4;
                releaseLoopCount++;
            }
            else
            {
                releaseLoopCount = 0;
                releaseStage++;
            }
            break;

        case 3:
            SetGoalAngle(NINETY_DEGREES);
            break;

    }
}