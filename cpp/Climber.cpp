#include "Common.h"
#include "Climber.h"

Climber::Climber(TalonXX* pRobot)
{
    encoder = new frc::Encoder(CLIMBER_ENCODER_ONE, CLIMBER_ENCODER_TWO, false);
    encoder->SetDistancePerPulse(CLIMBER_DISTANCE_PER_PULSE);
    climber = new frc::VictorSP(PWM_CLIMBER);

    mainRobot = pRobot;
    LocalReset();
    isCalibrating = true;
    encoderReset = true;
    calibrationCount = 0;
    
}

void Climber::LocalReset()
{
    motorCmd = 0.0;
    currentPosition = GetHeight();
    oldPosition = currentPosition;
    goalPosition = currentPosition;
    positionCmd = currentPosition;
    oldPCmd = currentPosition;

    positionError = 0.0;
    curVelocity = 0.0;
    atGoal = false;
    velocityCmd = 0.0;
    velocityError = 0.0;
    velocityErrorInt = 0.0;
    isManualMove = false;
    climbStage = 0;

    isEncoderBroken = false;
    encoderCheckCount = 0;
    initialEncoderReading = 0;
    encoderCheckFirstTime = true;
    isManualOverride = false;
    atGoalCount = 0;
    printCount = 0;
    isClimbing = false;
    isLevTwoClimbing = false;
    
    togetherClimbGoalPosition = 0.0;
    climbCount = 0;
    climber->Set(motorCmd);
}

void Climber::StartingConfig()
{

}

void Climber::StopAll()
{
    LocalReset();
}

double Climber::GetHeight()
{
    return encoder->GetDistance();
}

void Climber::ManualMove(double elevatorCmd)
{
	if (fabs(elevatorCmd) >= CLIMBER_MANUAL_DEADBAND)
	{
        if (elevatorCmd > 0.0)
		{
            if(!isManualMove)
            {
                goalPosition = currentPosition;
                isManualMove = true;
            }
            else
            {
                goalPosition = goalPosition + CLIMBER_UP_MANUAL_DELTA;
            }
		}
		else
		{
            if(!isManualMove)
            {
                goalPosition = currentPosition;
                isManualMove = true;
            }
            else
            {
                goalPosition = goalPosition + CLIMBER_DOWN_MANUAL_DELTA;
            }
		}
	}
	else
	{
        return;
	}
}

void Climber::OverrideManualMove(double overrideCmd)
{
    if(overrideCmd > 0.2)
    {
        motorCmd = 0.6;
    }
    else if(overrideCmd < -0.2)
    {
        motorCmd = -0.6;
    }
    else
    {
        motorCmd = 0.0;
    }
}

bool Climber::isOverrideOn()
{
    return isManualOverride;
}

void Climber::GiveGoalPosition(double height)
{
    isManualMove = false;
    goalPosition = height;
}

void Climber::ControlMove()
{
    if(calibrationCount > CLIMBER_CALIBRATION_COUNT)
    {
        if((int) (currentPosition * 1000) == (int) (oldPosition * 1000))
        {
            isCalibrating = false;
            if(encoderReset)
            {
                encoder->Reset();
                encoderReset = false;
                velocityErrorInt = 0.0;
                LocalReset();

            }
        }
    }
    else
    {
        calibrationCount++;
    }
    double downDeltaPos, upDeltaPos;
    if(isLevTwoClimbing)
    {
        downDeltaPos = -3.0 * LOOPTIME;
    }
    else
    {
        downDeltaPos = (-5.0 - 10.0*(currentPosition < 16))*LOOPTIME;
    }
    
    upDeltaPos = 5.0 * LOOPTIME;
    positionCmd = positionCmd + TalonXX::Limit(downDeltaPos, upDeltaPos, goalPosition - positionCmd);
	positionError = positionCmd + CLIMBER_KFF * ((positionCmd - oldPCmd)/LOOPTIME) - currentPosition;
    curVelocity = (currentPosition - oldPosition)/LOOPTIME;
    curVelocity = TalonXX::Limit(CLIMBER_MIN_VELOCITY, CLIMBER_MAX_VELOCITY, curVelocity);

    if(isCalibrating)
    {
        velocityCmd = -2.0;
    }
    else
    {
        if(fabs(goalPosition-currentPosition)<=ELEVATOR_ALLOWANCE)
        {
            velocityCmd = 0.0;
            atGoal = true;
        }

        else
        {
            velocityCmd = CLIMBER_KP * positionError;
            atGoal = false;
        }

    }
    
	velocityError = velocityCmd - curVelocity;
	velocityErrorInt = velocityErrorInt + (velocityError * LOOPTIME);
	velocityErrorInt = TalonXX::Limit(CLIMBER_MIN_VELOCITY_ERROR_INT, CLIMBER_MAX_VELOCITY_ERROR_INT, velocityErrorInt);

	motorCmd = CLIMBER_K_BANDWIDTH/CLIMBER_K * (CLIMBER_TAU * velocityError + velocityErrorInt);
    if(isCalibrating)
    {
        motorCmd = TalonXX::Limit(-0.15, 0.15, motorCmd);
    }
	oldPosition = currentPosition;
	oldPCmd = positionCmd;
}

double Climber::GetGoalPosition()
{
    return goalPosition;
}

bool Climber::AtFloor()
{
    if(currentPosition <= CLIMBER_BOTTOM_STOP_HEIGHT)
    {
        return(true);
    }
    return(false);
}

bool Climber::AtMax()
{
    if(currentPosition >= CLIMBER_TOP_STOP_HEIGHT)
    {
        return(true);
    }
    return(false);
}

bool Climber::AtGoalPosition()
{
    if(fabs(goalPosition - positionCmd) <= CLIMBER_ALLOWANCE)
	{
		atGoal = true;
	}
	else
	{
		atGoal = false;
	}
	return atGoal;
}

void Climber::TurnOnOverride()
{
    isManualOverride = true;
}

void Climber::TurnOffOverride()
{
    isManualOverride = false;
}
void Climber::ClimbStop()
{
    LocalReset();
}

void Climber::AutoClimb()
{
    switch(climbStage)
    {
        case 0:
            mainRobot->forcedDrive = 0.0; mainRobot->forcedRotate = 0.0; mainRobot->strafeCmd = 0.0;
            mainRobot->loopCount = 0.0;
            mainRobot->drive->GyroOn();
            climbStage++;
            break;

        case 1:
            mainRobot->forcedDrive = CLIMB_DRIVE_CMD; mainRobot->forcedRotate = 0.0; mainRobot->strafeCmd = 0.0;
            if(currentPosition < CLIMBER_TOP_STOP_HEIGHT)
            {
                ManualMove(1.0);
			    mainRobot->lift->ManualMove(0.11);
                
            }
            else
            {
                climbStage++;
                //mainRobot->theClaw->SetGoalAngle(UP_ANGLE);   // go to lesser angle here
            }
            break;

        case 2:
            climbCount++;
            mainRobot->forcedDrive = CLIMB_DRIVE_CMD; mainRobot->forcedRotate = 0.0; mainRobot->strafeCmd = 0.0;
            if(mainRobot->theClaw->GetAngle() > UP_ANGLE)
            {
                mainRobot->theClaw->ManualRotate(-0.11);
            }
            else
            {
                climbStage++;

            }
            if(climbCount > TWO_SEC)
            {
                GiveGoalPosition(CLIMBER_BOTTOM_STOP_HEIGHT);
            }
            break;
            
        case 3:  
      
            mainRobot->forcedDrive = CLIMB_DRIVE_CMD; mainRobot->forcedRotate = 0.0; mainRobot->strafeCmd = 0.0; //0.25
            break;
            
    }
}

void Climber::LevTwoAutoClimb()
{
    climbCount++;
    mainRobot->forcedDrive = CLIMB_DRIVE_CMD; mainRobot->forcedRotate = 0.0; mainRobot->strafeCmd = 0.0;

    switch(climbStage)
    {
        case 0:
            mainRobot->loopCount = 0.0;
            mainRobot->drive->GyroOn();
            climbStage++;
            break;

        case 1:
            if(mainRobot->theClaw->GetAngle() < CLIMB_TILT_ANGLE)
            {
                mainRobot->theClaw->ManualRotate(0.5);
            }
            else
            {
                climbCount = 0;
                climbStage++;
            }
            break;

        case 2:
            if(climbCount < HALF_SEC)
            {
                mainRobot->forcedDrive = 0.6;
            }
            else
            {
                climbCount = 0;
                climbStage++;
            }
            break;

        case 3:
            mainRobot->theClaw->SetGoalAngle(LEV_TWO_CLIMB_DRIVE_ANGLE);
            if(currentPosition < CLIMBER_LEV_TWO_CLIMB)
            {
                ManualMove(1.0);
            }
            else
            {
                mainRobot->theClaw->SetGoalAngle(FULL_BACK_POS);
                climbCount = 0;
                climbStage++;
            }
            break;

        case 4:
            if(climbCount > ONE_SEC)
            {
                climbCount = 0;
                climbStage++;
            }
            break;

        case 5:
            GiveGoalPosition(CLIMBER_MIN_HEIGHT);
            climbCount = 0;
            break;
            
            
    }
}

void Climber::TurnOnAutoClimb()
{
    if(!isClimbing)
    {
        isClimbing = true;
        climbStage = 0;
        climbCount = 0;
    }
}

void Climber::TurnOffAutoClimb()
{
    isClimbing = false;
}

void Climber::TurnOnLevTwoClimb()
{
    if(!isLevTwoClimbing)
    {
        isLevTwoClimbing = true;
        climbStage = 0;
        climbCount = 0;
    }
}

void Climber::TurnOffLevTwoClimb()
{
    isLevTwoClimbing = false;
}

void Climber::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Climber Motor Cmd: ", motorCmd);
    frc::SmartDashboard::PutNumber("Climber Goal Pos: ", goalPosition);
    frc::SmartDashboard::PutNumber("Climber Current Pos: ", currentPosition);
   // frc::SmartDashboard::PutBoolean("ClimberAtMax", AtMax());
   // frc::SmartDashboard::PutBoolean("ClimberAtFloor", AtFloor());
   // frc::SmartDashboard::PutBoolean("ClimberAtGoalPosition", AtGoalPosition());
    //frc::SmartDashboard::PutBoolean("ClimberisManualMove", isManualMove);
    frc::SmartDashboard::PutBoolean("ClimberIsOverride: ", isOverrideOn());

   // frc::SmartDashboard::PutBoolean("isClimberEncoderBroken", isEncoderBroken);
}

void Climber::Service()
{
   currentPosition = GetHeight();
    goalPosition = TalonXX::Limit(CLIMBER_BOTTOM_STOP_HEIGHT, CLIMBER_TOP_STOP_HEIGHT, goalPosition);
 
    if(isClimbing)
    {
        AutoClimb();
    }
    if(isLevTwoClimbing)
    {
        LevTwoAutoClimb();
    }
    if(!isManualOverride)
    {
        ControlMove();
    }
    
    motorCmd = TalonXX::Limit(CLIMBER_MIN_CMD, CLIMBER_MAX_CMD, motorCmd);

   if(mainRobot->isPaused && !isCalibrating)
    {
        climber->Set(0.0);
    }
    else
    {
        climber->Set(motorCmd);
    } 
}
