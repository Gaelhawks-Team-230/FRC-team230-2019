#include "frc/WPILib.h"
#include "Common.h"
#include "HatchHandler.h"
#include <frc/VictorSP.h>

HatchHandler::HatchHandler(TalonXX* pRobot)
{
    mainRobot = pRobot;
    cams= new frc::VictorSP(PWM_HATCH_CAMS);
    hatchSensor= new frc::DigitalInput(HATCH_CAM_SENSOR);
#ifdef USE_VACUUM
    vacuum = new frc::VictorSP(PWM_VACUUM);
    wasVacOn = false;
#endif
    LocalReset();
}

void HatchHandler::LocalReset()
{
#ifdef USE_VACUUM
    if(wasVacOn)
    {
        StartVacuum();
    }
    else
    {
        ReleaseVacuum();
    }
#endif
    camMotorCmd =0.0;
    resetCount=0;
    isReleasing=false;
    isResetting=false;
    loopCount = 0;
    stage = 0;
    loopStage = 0;
    climbTurnLoopCount = 0;
    isFeederStation = false;
    feederStationStage = 0;
    feederStationCount = 0;
    floorPickupStage = 0;
    isFloorPickup = false;
    camDownLoopCount = 0;
    camDownLoopStage = 0;
    isCamDown = false;
    camDownStage = 0;
    currentReading = false;
    oldReading = true;
    firstTimeThrough = true;
    SeeCam();
#ifdef USE_VACUUM
    vacFloorPickupCount = 0;
#endif
}

void HatchHandler::StartingConfig()
{
    LocalReset();
}

void HatchHandler::StopAll()
{
#ifdef USE_VACUUM
    wasVacOn = vacOn;
#endif
    LocalReset();
#ifdef USE_VACUUM
    ReleaseVacuum();   // force it off after saving the current state
#endif
}

void HatchHandler::MoveToFloor()
{
    mainRobot->theClaw->SetGoalAngle(BOTTOM_STOP);
}

void HatchHandler::PickupHatch()
{
    mainRobot->theClaw->SetGoalAngle(TOP_STOP);
}

void HatchHandler::ReleaseHatch()
{
    if(!isReleasing || stage==2)
    {
        loopCount = 0;
        isReleasing = true;
        stage = 0;
    }
}

void HatchHandler::StopReleaseHatch()
{
    camMotorCmd=0.0;
    isReleasing=false;
    isResetting=false;
#ifdef USE_VACUUM
    ReleaseVacuum();
#endif
}

void HatchHandler::ResetCams()
{
    camMotorCmd=-0.2;
    isReleasing=false;
    isResetting=true;
}


void HatchHandler::FloorPickUp()
{
    switch(floorPickupStage)
    {
        case 0:
#ifdef USE_VACUUM
            StartVacuum();
#endif
            mainRobot->theClaw->SetGoalAngle(TOP_MAX);
            if(mainRobot->lift->GetHeight() < FLOOR_PICKUP_HEIGHT)
            {
                mainRobot->lift->ManualMove(ELEVATOR_UP_MANUAL_MOVE);
            }
            else
            {
                floorPickupStage++;
            }
            break;

        case 1:
            mainRobot->theClaw->SetGoalAngle(FLOOR_PICKUP_UP_ANGLE);
            mainRobot->lift->GiveGoalPosition(ELEVATOR_BOTTOM_STOP_HEIGHT);
#ifdef USE_VACUUM
            floorPickupStage++;
#else
            floorPickupStage = 0;
            isFloorPickup = false;
#endif
            break;

#ifdef USE_VACUUM
        case 2:
            if(mainRobot->theClaw->AtGoalAngle())
            {
                HoldVacuum();
                floorPickupStage = 0;
                isFloorPickup = false;
            }           
            break;
#endif

    }
}
bool HatchHandler::SeeCam()
{
    if(hatchSensor->Get())
    {
        seesCam= true;
    }
    else
    {
        seesCam= false;
    }
}

void HatchHandler::TurnOnFloorPickup()
{
    if (!isFloorPickup)
    {
        isFloorPickup = true;
        floorPickupStage = 0;
    }
}

void HatchHandler::TurnOnFeederStationRelease()
{
    if (!isFeederStation)
    {
        isFeederStation = true;
        feederStationCount = 0;
        feederStationStage = 0;
    }
}

// Need to have this to prepare it to be able to restart if button is pushed again...
// before it has completed the release operation after a failed attempt to grab it a hatch
void HatchHandler::TurnOffFeederStationRelease()
{
    if (isFeederStation)
    {
        isFeederStation = false;
        feederStationCount = 0;
        feederStationStage = 0;
    }
}

void HatchHandler::AutomaticHatchRelease()
{
    currentReading = seesCam;
    loopCount++;
#ifdef USE_VACUUM
    ReleaseVacuum();
#endif
    switch(stage)
    {
        case 0:
            if(loopCount < EJECT_TIME)
            {
                camMotorCmd = -1.0;
                if (loopCount < EJECT_BACKUP)
                {
                    mainRobot->forcedDrive = -0.25;
                }
                else
                {
                    mainRobot->forcedDrive = 0.0;
                }
            }
            else
            {
                loopCount = 0;
                stage++;
            }
            break;

        case 1:
            if(loopCount < WAIT_TIME)
            {
                loopStage = (loopCount % COMMAND_ARRAY_SIZE);

                camMotorCmd = cmdArray[loopStage];
             }
            else
            {
                loopCount = 0;
                stage++;
            }
            break;

        case 2:          
            if(firstTimeThrough)
            {
                loopStage = (loopCount % COMMAND_ARRAY_SIZE);
                firstTimeThrough = false;
                camMotorCmd = cmdArray[loopStage];
                //printf("FirstPass\n");
            }
            else if(!currentReading && oldReading)
            {
                camMotorCmd = 0.0;
                isReleasing = false;
                firstTimeThrough = true;

                loopStage = 0;
                loopCount = 0;
                stage = 0;
            }
            else
            {
                loopStage = (loopCount % COMMAND_ARRAY_SIZE);
                camMotorCmd = cmdArray[loopStage];
             }
             break;   
    }
    oldReading = currentReading;
   
}
void HatchHandler::FeederStation()
{
    if(isFeederStation)
    {
        switch(feederStationStage)
		{
			case 0:
				mainRobot->lift->GiveGoalPosition(ELEVATOR_HEIGHT_FEEDER_STATION);
				//mainRobot->theClaw->SetGoalAngle(FULL_BACK_POS);
				feederStationCount = 0;
				feederStationStage++;
				break;

			case 1:
				//printf("Case 1\n");
				if(feederStationCount > FEEDER_STATION_DRIVE_BACK_COUNT)
				{
					feederStationCount = 0;
					feederStationStage++;
					mainRobot->distance->ResetEncoders();
				}
				else
				{
					feederStationCount++;
				}
				break;

			case 2:
				//printf("Case 2\n");
				if((mainRobot->distance->GetForwardsDistance() * -1.0) < FEEDER_STATIOn_DRIVE_BACK_DISTANCE)
				{
					mainRobot->forcedDrive = -0.5; mainRobot->forcedRotate = 0.0; mainRobot->forcedStrafe = 0.0;
				}
				else
				{
					mainRobot->forcedDrive = 0.0; mainRobot->forcedRotate = 0.0; mainRobot->forcedStrafe = 0.0;
					feederStationStage++;
				}
				break;
				
			case 3:
				mainRobot->theClaw->SetGoalAngle(NINETY_DEGREES);
				//mainRobot->lift->GiveGoalPosition(ELEVATOR_BOTTOM_STOP_HEIGHT);
				feederStationStage = 0;
                isFeederStation = false;
				break;
		}
    }
}

void HatchHandler::UpdateDash()
{
  //  frc::SmartDashboard::PutNumber("camMotorCmd: ", camMotorCmd);
    frc::SmartDashboard::PutBoolean("seesCam", seesCam);
#ifdef USE_VACUUM
    frc::SmartDashboard::PutBoolean("vacOn", vacOn);
    frc::SmartDashboard::PutNumber("vacMotorCmd", vacMotorCmd);
#endif
   // frc::SmartDashboard::PutNumber("camResetCount", resetCount);
  //  frc::SmartDashboard::PutBoolean("CamisReleasing", isReleasing);
   // frc::SmartDashboard::PutBoolean("CamisResetting", isResetting);
}

//Called every loop
void HatchHandler::Service()
{
    if(mainRobot->isPaused)
    {
        cams->Set(0.0);
    }
    FeederStation();
    if(isFloorPickup)
    {
        FloorPickUp();
    }
    SeeCam();
    if(isReleasing)
    {
        AutomaticHatchRelease();
    }
   
    cams->Set(camMotorCmd);
#ifdef USE_VACUUM
    vacuum->Set(vacMotorCmd);
#endif
  
}

#ifdef USE_VACUUM
void HatchHandler::StartVacuum()
{
    vacMotorCmd = 0.5;
    vacOn = true;
}

void HatchHandler::HoldVacuum()
{
    vacMotorCmd = 0.2;
    vacOn = true;
}
void HatchHandler::ReleaseVacuum()
{
    vacMotorCmd = 0.0;
    vacOn = false;
}
#endif

