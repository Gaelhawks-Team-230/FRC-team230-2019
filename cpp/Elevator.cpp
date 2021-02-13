#include "Common.h"
#include "Elevator.h"

Elevator::Elevator(TalonXX* pRobot)
{
    encoder = new frc::Encoder(ELEVATOR_ENCODER_ONE, ELEVATOR_ENCODER_TWO, false);
    encoder->SetDistancePerPulse(LIFT_DISTANCE_PER_PULSE);
    elevator1 = new frc::VictorSP(PWM_ELEVATOR_ONE);

    mainRobot = pRobot;
    LocalReset();
    isCalibrating = true;
    encoderReset = true;
}

void Elevator::LocalReset()
{
    motorCmd = 0.0;
    currentPosition = GetHeight();
    oldPosition = currentPosition;
    goalPosition = currentPosition;
    positionCmd = currentPosition;
    holdGoalPosition = currentPosition;
    oldPCmd = currentPosition;

    positionError = 0.0;
    curVelocity = 0.0;
    atGoal = false;
    velocityCmd = 0.0;
    velocityError = 0.0;
    velocityErrorInt = 0.0;
    isManualMove = false;
    isManualOverride = false;

    
    atGoalCount = 0;

    elevator1->Set(motorCmd);
}

void Elevator::StartingConfig()
{

}

void Elevator::StopAll()
{
    LocalReset();
}

double Elevator::GetHeight()
{
    return encoder->GetDistance();
}

void Elevator::ManualMove(double elevatorCmd)
{
    elevatorCmd = elevatorCmd * -1.0;
	if (fabs(elevatorCmd) >= ELEVATOR_MANUAL_DEADBAND)
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
                goalPosition = goalPosition + (ELEVATOR_UP_MANUAL_DELTA * elevatorCmd);
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
                goalPosition = goalPosition - (ELEVATOR_DOWN_MANUAL_DELTA * elevatorCmd);
            }
		}
	}
	else
	{
		return;
	}
}

void Elevator::OverrideManualMove(double overrideCmd)
{
    if(fabs(overrideCmd) >= ELEVATOR_MANUAL_DEADBAND)
	{
		motorCmd = ELEVATOR_OVERRIDE_CONSTANT * overrideCmd;
	}
    else
    {
        motorCmd = 0.0;
    }
}

void Elevator::OverrideHold()
{
    motorCmd = ELEVATOR_HOLD_CMD;
}

void Elevator::TurnOnManualOverride()
{
    isManualOverride = true;
}

void Elevator::TurnOffManualOverride()
{
    isManualOverride = false;
    LocalReset();
}

bool Elevator::isOverrideOn()
{
    return isManualOverride;
}
void Elevator::GiveGoalPosition(double height)
{
    isManualMove = false;
    goalPosition = height;
}

bool Elevator::ReturnEncoderReset()
{
    return encoderReset;
}

void Elevator::ControlMove()
{
    if((int) (currentPosition * 1000) == (int) (oldPosition * 1000))
    {
        isCalibrating = false;
        if(encoderReset)
        {
            encoder->Reset();
            encoderReset = false;
            //printf("Encoder Rest\n");
        }
        //printf("Elevator Done Calibrating \n");
    }
	positionCmd = positionCmd + TalonXX::Limit(ELEVATOR_DOWN_MAX_DELTA_POS, ELEVATOR_UP_MAX_DELTA_POS, goalPosition - positionCmd);
	
    positionError = positionCmd + ELEVATOR_KFF * ((positionCmd - oldPCmd)/LOOPTIME) - currentPosition;
    curVelocity = (currentPosition - oldPosition)/LOOPTIME;
    curVelocity = TalonXX::Limit(ELEVATOR_MIN_VELOCITY, ELEVATOR_MAX_VELOCITY, curVelocity);
	
    if(!isCalibrating)
    {
        if(fabs(goalPosition-currentPosition)<=ELEVATOR_ALLOWANCE)
        {
           // velocityCmd = 0.0;
            atGoal = true;
        }
        else
        {
            //velocityCmd = ELEVATOR_KP * positionError;
            atGoal = false;
        }
        velocityCmd = ELEVATOR_KP * positionError;

    }
    else
    {
        velocityCmd = -2.0;
    }

	velocityError = velocityCmd - curVelocity;
	velocityErrorInt = velocityErrorInt + (velocityError * LOOPTIME);
//#ifdef PRACTICE_BOT
    double velErrIntMin;
    velErrIntMin = ELEVATOR_MIN_VELOCITY_ERROR_INT*TalonXX::Limit(0.0,1.0, currentPosition);
    velocityErrorInt = TalonXX::Limit(velErrIntMin, ELEVATOR_MAX_VELOCITY_ERROR_INT, velocityErrorInt);
//#else 
  //  velocityErrorInt = TalonXX::Limit(ELEVATOR_MIN_VELOCITY_ERROR_INT, ELEVATOR_MAX_VELOCITY_ERROR_INT, velocityErrorInt);
//#endif

	motorCmd = ELEVATOR_K_BANDWIDTH/ELEVATOR_K * (ELEVATOR_TAU * velocityError + velocityErrorInt);
	oldPosition = currentPosition;
	oldPCmd = positionCmd;
}

double Elevator::GetGoalPosition()
{
    return goalPosition;
}


bool Elevator::AtFloor()
{
    if(currentPosition <= ELEVATOR_BOTTOM_STOP_HEIGHT)
    {
        return(true);
    }
    return(false);
}

bool Elevator::AtMax()
{
    if(currentPosition >= ELEVATOR_TOP_STOP_HEIGHT)
    {
        return(true);
    }
    return(false);
}

bool Elevator::AtGoalPosition()
{
    if(fabs(goalPosition - currentPosition) <= ELEVATOR_AT_GOAL_ALLOWANCE)
	{
		atGoal = true;
	}
	else
	{
		atGoal = false;
	}
	return atGoal;
}

void Elevator::LiftStop()
{
    LocalReset();
}

void Elevator::UpdateDash()
{
    frc::SmartDashboard::PutNumber("Elevator Motor Cmd: ", motorCmd);
    frc::SmartDashboard::PutNumber("Elevator Goal Pos: ", goalPosition);
    frc::SmartDashboard::PutNumber("Elevator Current Pos: ", currentPosition);
    //frc::SmartDashboard::PutBoolean("ElevatorAtMax", AtMax());
   // frc::SmartDashboard::PutBoolean("ElevatorAtFloor", AtFloor());
  //  frc::SmartDashboard::PutBoolean("ElevatorAtGoalPosition", AtGoalPosition());
   // frc::SmartDashboard::PutBoolean("ElevatorisManualMove", isManualMove);
    frc::SmartDashboard::PutBoolean("ElevatorIsOverride: ", isManualOverride);
  //  frc::SmartDashboard::PutNumber("Test: ", 1)
}

void Elevator::Service()
{
    currentPosition = GetHeight();

    if(!isCalibrating)
    {
        goalPosition = TalonXX::Limit(ELEVATOR_BOTTOM_STOP_HEIGHT, ELEVATOR_MAX_HEIGHT, goalPosition);
    }
    motorCmd = TalonXX::Limit(ELEVATOR_MIN_CMD, ELEVATOR_MAX_CMD, motorCmd);

    if(!isManualOverride)
    {
        ControlMove();
       // printf("%f %f %f \n", motorCmd, positionCmd, currentPosition);

    }

    if(positionCmd >= TILT_CHECK_HEIGHT_LOW && positionCmd <= TILT_CHECK_HEIGHT_HIGH && mainRobot->theClaw->GetAngle()<PIXY_PROTECTION_ANGLE)
    {
        holdGoalPosition = goalPosition;
        goalPosition = currentPosition;
        mainRobot->theClaw->SetGoalAngle(NINETY_DEGREES);
        if(mainRobot->theClaw->AtGoalAngle())
        {
            goalPosition = holdGoalPosition;
        }
    }

    if(mainRobot->isPaused && !isCalibrating)
    {
        elevator1->Set(0.0);
    }
    else
    {
        elevator1->Set(motorCmd);
    } 

}
