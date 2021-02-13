#include "Common.h"
#include "Autonomous.h"
#include "TalonXX_main.h"


void TalonXX::DoNothing()
{
    //printf("DO NOTHING\n");
    driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
    loopCount = 0;
    autoStage = 0;
}

void TalonXX::FullManual()
{
    //printf("FULL MANUAL\n");
    JoystickButtonHandler();
    isAuto = false;
    isAutoManualOverride = true;
}

void TalonXX::BaselineFromCenter()
{
    //printf("BASELINE FROM CENTER\n");
    switch(autoStage)
    {
        case 0: //Reset
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
            distance->LocalReset();
            loopCount = 0;
            autoStage++;
            break;

        case 1:
            if(distance->GetForwardsDistance() < BASELINE_DISTANCE_FROM_CENTER/2)
            {
                driveCmd = driveCmd + Limit(MIN_PLATFORM_ACCELERATION_CENTER, MAX_PLATFORM_ACCELERATION_CENTER, 0.6 - driveCmd);
                rotateCmd = 0.0; strafeCmd = 0.0;
                loopCount++;
            }
            else if(distance->GetForwardsDistance() < BASELINE_DISTANCE_FROM_CENTER)
            {
                driveCmd = driveCmd + Limit(MIN_PLATFORM_ACCELERATION_CENTER, MAX_PLATFORM_ACCELERATION_CENTER, 0.2 - driveCmd);
                rotateCmd = 0.0; strafeCmd = 0.0;
                loopCount++;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				//distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
            }
            break;

        case 2:
            autoMode = 1;
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
    }
    rotateCmd*=autoRotateMultiplier;
    strafeCmd*=autoStrafeMultiplier;
}

void TalonXX::BaselineFromSide()
{
    //printf("BaselineFromSide\n");
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
            distance->LocalReset();
            loopCount = 0;
            autoStage++;
            break;
        
        case 1:
            if(distance->GetForwardsDistance() < BASELINE_DISTANCE_FROM_SIDE)
            {
                driveCmd = 0.45;
                rotateCmd = 0.0; strafeCmd = 0.0;
                loopCount++;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
            }
            break;

        case 2:
            autoMode = 1;
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
    }
    rotateCmd*=autoRotateMultiplier;
    strafeCmd*=autoStrafeMultiplier;
}

void TalonXX::EndHatchPanelFromCenter()
{
    //printf("EndHatchPanelFromCenter\n");
#ifdef USE_PIXY
    loopCount++;
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
            distance->LocalReset();
            loopCount = 0;
            theClaw->SetGoalAngle(NINETY_DEGREES);
            autoStage++;
            break;

        case 1:
            isTracking = true;
            if(distance->GetForwardsDistance() < DISTANCE_OFF_CENTER)
            {
                driveCmd = driveCmd + Limit(MIN_PLATFORM_ACCELERATION_CENTER, MAX_PLATFORM_ACCELERATION_CENTER, 0.4 - driveCmd);
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				loopCount = 0;
				autoStage++;
            }
            break;

        case 2:
            if(distance->GetForwardsDistance() < END_HATCH_PANEL_FROM_CENTER_DISTANCE*0.75)
            {
                driveCmd = driveCmd + Limit(MIN_PLATFORM_ACCELERATION_CENTER, MAX_PLATFORM_ACCELERATION_CENTER, 0.4 - driveCmd);
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < END_HATCH_PANEL_FROM_CENTER_DISTANCE)
            {
                driveCmd = driveCmd + Limit(MIN_PLATFORM_ACCELERATION_CENTER, MAX_PLATFORM_ACCELERATION_CENTER, 0.2 - driveCmd);
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				loopCount = 0;
				autoStage++;
            }
            if(seesTarget || isLineDetected)
            {
                autoStage++;
            }
            break;

        case 3:
            if(distance->GetForwardsDistance() < DISTANCE_FOLLOWING_LINE)
            {
                strafeCmd = DoTracking(false, true, isAuto);
                if(isLineDetected)
                {
                    driveCmd = 0.2; rotateCmd = 0.0;
                }
                else
                {
                    driveCmd = 0.3; rotateCmd = 0.0;
                }
                
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				loopCount = 0;
				autoStage++;
            }
            if(loopCount > TIME_CHECK_FOR_TARGETTING)
            {
                //printf("isLineDetected Time Check\n");
                isTracking = false;
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				loopCount = 0;
				autoStage++;            
            }
            break;

        case 4:
            if(distance->GetForwardsDistance() < FAST_FORWARD_DISTANCE)//Wait might be necessary
            {
                //printf("Fast Forward\n");
                driveCmd = 0.4; strafeCmd = 0.0;
                rotateCmd = (LINE_CENTER_VALUE - lineFollower->LineLocation()) * LINE_ROTATE_MULTIPLIER;
            }
            else
            {
                //printf("Release Hatch\n");
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
                hatchPanel->ReleaseHatch();
                distance->ResetEncoders();
                loopCount = 0;
                autoStage++;
            }         
            if(loopCount > RELEASE_TIME_LIMIT)
            {
                //printf("Release Time Check after fast\n");
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
                hatchPanel->ReleaseHatch();
                 distance->ResetEncoders();
                loopCount = 0;
                autoStage++;
            }
               
            break;

        case 5:
            if((distance->GetForwardsDistance() * -1.0) < BACK_DISTANCE_AFTER_PLACEMENT_FROM_END_CENTER_MODE)
            {
                //printf("Driving back %f\n", distance->GetForwardsDistance());
                driveCmd = -0.2; strafeCmd = 0.0;
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
 				loopCount = 0;
				autoStage++;
            }
            break;
            
        case 6:
           // printf("Case 5\n");
            autoMode = 1;
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
            isTracking = false;

			break;
    }
    rotateCmd*=autoRotateMultiplier;
    strafeCmd*=autoStrafeMultiplier;
#endif
}

void TalonXX::EndHatchPanelFromSide()
{
    //printf("EndHatchPanelFromSide\n");
#ifdef USE_PIXY
    loopCount++;
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
            distance->LocalReset();
            loopCount = 0;
            autoStage++;
            break;
        
        case 1:
          // printf("Case 1 %f \n", distance->GetForwardsDistance());
            theClaw->SetGoalAngle(FEEDER_STATION_UP);
            if(distance->GetForwardsDistance() < END_HATCH_PANEL_FROM_SIDE_DISTANCE)
            {
                driveCmd = 0.55;
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;// printf("loopcount = 0 case 1\n");
				autoStage++;
            }
            break;

        case 2:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = 60.0; strafeCmd = 0.0;
               // printf("Turn Left %d \n", loopCount);
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 2\n");
				autoStage++;
            }
            break;

        case 3:
           // printf("Drive Forward %f \n", distance->GetForwardsDistance());
            if(distance->GetForwardsDistance() < DISTANCE_AFTER_TURN_SIDE_TO_END * 0.75)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.4 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < DISTANCE_AFTER_TURN_SIDE_AUTO)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 3\n");
				autoStage++;
            }
            break;

        case 4:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = -60.0; strafeCmd = 0.0;
               // printf("Turn Right %d \n", loopCount);
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 4\n");
				autoStage++;
            }
            break;
        
        case 5:
           // printf("Case 5 %f \n", distance->GetForwardsDistance());
            if(distance->GetForwardsDistance() < DISTANCE_INTO_END_BAY_FROM_SIDE * 0.75)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.6 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < DISTANCE_INTO_END_BAY_FROM_SIDE)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                isTracking = true;
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 5\n");
				autoStage++;
            }
            break;
            
        case 6:
            if(!isLineDetected)
            {
                driveCmd = 0.0; strafeCmd = 0.5; rotateCmd = 0.0;
             //   printf("Looking for line \n");
            }
            else
            {
                driveCmd = 0.2; strafeCmd = DoTracking(false, false, false); rotateCmd = 0.0;
             //   loopCount = 0; printf("loopcount = 0 case 7, %f\n", strafeCmd);
                autoStage++;
            }
           // printf("case 7 %d \n", loopCount);
            break;

        case 7:
            if(loopCount < ONE_SEC)
            {
              // printf("case 8 %d \n", loopCount);
                if(isLineDetected)
                {
                    driveCmd = 0.2; strafeCmd = DoTracking(false, false, false); rotateCmd = 0.0;
                    //printf("strafecmd: %f", strafeCmd);
                }
                else
                {
                    driveCmd = 0.2; strafeCmd = 0.0; rotateCmd = 0.0;
                }
            }
            else
            {
              //  printf("release hatch\n");
                driveCmd = 0.0; strafeCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0; //printf("loopcount = 0 case 8\n");
                hatchPanel->ReleaseHatch();
                distance->ResetEncoders();
                autoStage++;
            }
            break;

        case 8:
            if(distance->GetForwardsDistance() * -1 < DISTANCE_AWAY_FROM_END_BAY)
            {
                driveCmd = -0.2; strafeCmd = 0.0;
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; 
				autoStage++;
            }
            break;
    }
    rotateCmd*=autoRotateMultiplier;
    strafeCmd*=autoStrafeMultiplier;
#endif
}


void TalonXX::SideHatchPanel()
{
    //printf("SideHatchPanel\n");
#ifdef USE_PIXY
    double initialDriveDistance;
    double backDistance;
    double distanceToCommonEndPoint;
    
    loopCount++;
    initialDriveDistance = SIDE_HATCH_PANEL_NEAR_DISTANCE;
    backDistance = BACK_DISTANCE_AFTER_NEAR_BAY;
    distanceToCommonEndPoint = DISTANCE_TO_COMMON_END_POINT_NEAR;

    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
            isTracking = false;
            distance->LocalReset();
            loopCount = 0; //printf("loopcount = 0 case 0\n");
            autoStage++;
            break;
        
        case 1:
          // printf("Case 1 %f \n", distance->GetForwardsDistance());
            theClaw->SetGoalAngle(FEEDER_STATION_UP);
            if(distance->GetForwardsDistance() < DISTANCE_FOR_SIDE_PANEL_FROM_SIDE)
            {
                driveCmd = 0.55;
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;// printf("loopcount = 0 case 1\n");
				autoStage++;
            }
            break;

        case 2:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = -60.0; strafeCmd = 0.0;
               // printf("Turn Left %d \n", loopCount);
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 2\n");
				autoStage++;
            }
            break;

        case 3:
           // printf("Drive Forward %f \n", distance->GetForwardsDistance());
            if(distance->GetForwardsDistance() < DISTANCE_AFTER_TURN_SIDE_AUTO * 0.6)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.4 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < DISTANCE_AFTER_TURN_SIDE_AUTO)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 3\n");
				autoStage++;
            }
            break;

        case 4:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = 60.0; strafeCmd = 0.0;
               // printf("Turn Right %d \n", loopCount);
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 4\n");
				autoStage++;
            }
            
            break;
        
        case 5:
           // printf("Case 5 %f \n", distance->GetForwardsDistance());
            if(distance->GetForwardsDistance() < initialDriveDistance * 0.75)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.6 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < initialDriveDistance)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 5\n");
				autoStage++;
            }
            break;

        case 6:
          //  printf("Face Target %d \n", loopCount);
            if(loopCount < ONE_SEC)
            {
                driveCmd = 0.0; rotateCmd = 90.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
                lift->GiveGoalPosition(ELEVATOR_CARGO_SHIP_HATCH_HEIGHT);
				distance->ResetEncoders();
                isTracking = true;
				loopCount = 0;// printf("loopcount = 0 case 6\n");
				autoStage++;
            }
            break;
            
        case 7:
            if(!isLineDetected)
            {
                driveCmd = 0.0; strafeCmd = -0.5; rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.2; strafeCmd = DoTracking(false, false, false); rotateCmd = 0.0;
                loopCount = 0; //printf("loopcount = 0 case 7\n");
                autoStage++;
            }
           // printf("case 7 %d \n", loopCount);
            break;

        case 8:
            if(loopCount < TWO_SEC)
            {
              // printf("case 8 %d \n", loopCount);
                if(isLineDetected)
                {
                    driveCmd = 0.2; strafeCmd = DoTracking(false, false, false); rotateCmd = 0.0;
                }
                else
                {
                    driveCmd = 0.2; strafeCmd = 0.0; rotateCmd = 0.0;
                }
            }
            else
            {
              //  printf("release hatch\n");
                driveCmd = 0.0; strafeCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0; //printf("loopcount = 0 case 8\n");
                hatchPanel->ReleaseHatch();
                distance->ResetEncoders();
                autoStage++;
            }
            break;

        case 9:
          //  printf("case 9: %d\n", loopCount);
            if(distance->GetForwardsDistance() * -1 < backDistance)
            {
                driveCmd = -0.2; strafeCmd = 0.0;
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 9\n");
				autoStage++;
            }
            break;

        case 10:
            if(loopCount < ONE_SEC)
            {
                driveCmd = 0.0; rotateCmd = 120.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 10\n");
				autoStage++;
            }
            break;

        case 11: 
            if(distance->GetForwardsDistance() < distanceToCommonEndPoint * 0.75)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.6 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < distanceToCommonEndPoint)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
            }
            break;

        case 12:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = -60.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
            }
            break;

        case 13:
            autoMode = 1;
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
    }
    rotateCmd*=autoRotateMultiplier;
    strafeCmd*=autoStrafeMultiplier;
#endif
}

void TalonXX::FarRocket()
{
    //printf("FarRocket\n");
#ifdef USE_PIXY    
    loopCount++;
    switch (autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
            isTracking = false;
            distance->LocalReset();
            loopCount = 0; //printf("loopcount = 0 case 0\n");
            autoStage++;
            break;
        
        case 1:
          // printf("Case 1 %f \n", distance->GetForwardsDistance());
            theClaw->SetGoalAngle(FEEDER_STATION_UP);
            if(distance->GetForwardsDistance() < DISTANCE_OFF_LEV_2_FOR_ROCKET)
            {
                driveCmd = 0.55;
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;// printf("loopcount = 0 case 1\n");
				autoStage++;
            }
            break;

        case 2:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = -70.0; strafeCmd = 0.0;
               // printf("Turn Left %d \n", loopCount);
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 2\n");
				autoStage++;
            }
            break;

        case 3:
           // printf("Drive Forward %f \n", distance->GetForwardsDistance());
            if(distance->GetForwardsDistance() < DISTANCE_TO_ROCKET_AFTER_TURN * 0.6)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.4 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < DISTANCE_TO_ROCKET_AFTER_TURN)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 3\n");
				autoStage++;
            }
            break;

        case 4:
            if(loopCount < ONE_SEC)
            {
                driveCmd = 0.0; rotateCmd = -110.0; strafeCmd = 0.0;
               // printf("Turn Right %d \n", loopCount);
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
                isTracking = true;
				loopCount = 0; //printf("loopcount = 0 case 4\n");
				autoStage++;
            }
            break;

        case 5:
            if(distance->GetForwardsDistance() < DISTANCE_INTO_ROCKET)
            {
                driveCmd = 0.3; 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 3\n");
				autoStage++;
            }
            break;

        case 6:
            if(!isLineDetected)
            {
                driveCmd = 0.0; strafeCmd = 0.5; rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; strafeCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0;
                autoStage++;
            }
            break;

        case 7:
            autoMode = 1;
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
    }
    rotateCmd *= autoRotateMultiplier;
    strafeCmd *= autoStrafeMultiplier;
#endif
}

void TalonXX::NearRocket()
{
    //printf("NearRocket\n");
#ifdef USE_PIXY
    loopCount++;
    switch(autoStage)
    {
        case 0:
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
            isTracking = false;
            distance->LocalReset();
            loopCount = 0; //printf("loopcount = 0 case 0\n");
            autoStage++;
            break;
        
        case 1:
          // printf("Case 1 %f \n", distance->GetForwardsDistance());
            theClaw->SetGoalAngle(FEEDER_STATION_UP);
            if(distance->GetForwardsDistance() < DISTANCE_OFF_PLATFORM_NEAR_ROCKET)
            {
                driveCmd = 0.55;
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;// printf("loopcount = 0 case 1\n");
				autoStage++;
            }
            break;

        case 2:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = -100.0; strafeCmd = 0.0;
               // printf("Turn Left %d \n", loopCount);
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 2\n");
				autoStage++;
            }
            break;

        case 3:
           // printf("Drive Forward %f \n", distance->GetForwardsDistance());
            if(distance->GetForwardsDistance() < DISTANCE_TO_NEAR_ROCKET * 0.6)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.4 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < DISTANCE_TO_NEAR_ROCKET)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 3\n");
				autoStage++;
            }
            break;

        case 4:
            if(loopCount < HALF_SEC)
            {
                driveCmd = 0.0; rotateCmd = 60.0; strafeCmd = 0.0;
               // printf("Turn Right %d \n", loopCount);
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 4\n");
				autoStage++;
            }
            
            break;
        
        case 5:
           // printf("Case 5 %f \n", distance->GetForwardsDistance());
            if(distance->GetForwardsDistance() < DISTANCE_INTO_NEAR_ROCKET * 0.75)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.6 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < DISTANCE_INTO_NEAR_ROCKET)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                isTracking = true;
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 5\n");
				autoStage++;
            }
            break;
            
        case 6:
            if(!isLineDetected)
            {
                driveCmd = 0.0; strafeCmd = -0.5; rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.2; strafeCmd = DoTracking(false, false, false); rotateCmd = 0.0;
                loopCount = 0; //printf("loopcount = 0 case 7\n");
                autoStage++;
            }
           // printf("case 7 %d \n", loopCount);
            break;

        case 7:
            if(loopCount < ONE_SEC)
            {
              // printf("case 8 %d \n", loopCount);
                if(isLineDetected)
                {
                    driveCmd = 0.2; strafeCmd = DoTracking(false, false, false); rotateCmd = 0.0;
                }
                else
                {
                    driveCmd = 0.2; strafeCmd = 0.0; rotateCmd = 0.0;
                }
            }
            else
            {
              //  printf("release hatch\n");
                driveCmd = 0.0; strafeCmd = 0.0; rotateCmd = 0.0;
                loopCount = 0; //printf("loopcount = 0 case 8\n");
                hatchPanel->ReleaseHatch();
                distance->ResetEncoders();
                autoStage++;
            }
            break;

        case 8:
          //  printf("case 9: %d\n", loopCount);
            if(distance->GetForwardsDistance() * -1 < DISTANCE_AWAY_FROM_NEAR_ROCKET)
            {
                driveCmd = -0.2; strafeCmd = 0.0;
                rotateCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 9\n");
				autoStage++;
            }
            break;

        case 9:
            if(loopCount < ONE_P_FIVE_SEC)
            {
                driveCmd = 0.0; rotateCmd = -100.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0; //printf("loopcount = 0 case 10\n");
				autoStage++;
            }
            break;

        case 10: 
            if(distance->GetForwardsDistance() < DISTANCE_TO_FEEDER_STATION * 0.75)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.6 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else if(distance->GetForwardsDistance() < DISTANCE_TO_FEEDER_STATION)
            {
                driveCmd = driveCmd + TalonXX::Limit(AUTO_DRIVE_MIN_ACCELERATION, AUTO_DRIVE_MAX_ACCELERATION, 0.2 - driveCmd); 
                rotateCmd = 0.0; strafeCmd = 0.0;
            }
            else
            {
                driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
				distance->ResetEncoders();
				loopCount = 0;
				autoStage++;
            }
            break;

        case 11:
            autoMode = 1;
            driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
			loopCount = 0;
			autoStage = 0;
			break;
    }
    rotateCmd *= autoRotateMultiplier;
    strafeCmd *= autoStrafeMultiplier;
#endif
}

void TalonXX::AutoMultipliers()
{
    if(autoStartPosition == LEFT_POS)
    {
        autoRotateMultiplier = 1.0;
        autoStrafeMultiplier = 1.0;
    }
    else if(autoStartPosition == RIGHT_POS)
    {
        autoRotateMultiplier = -1.0;
        autoStrafeMultiplier = -1.0;
    }
}

void TalonXX::ModeSelection()
{
    bool modeChange = false;
    
    int tempStartPos = autoStartPosition;
    int tempL1Bay = autoL1Bay;

    autoStartPosition = (AutoPositionChooser->GetSelected());
    autoL1Bay = (LevelOneBay->GetSelected());

    if(autoStartPosition == DO_NOTHING)
    {
        autoMode = 0;
    }
    else if(autoStartPosition == FULL_MANUAL)
    {
        autoMode = 1;
    }
    else if(autoStartPosition == CENTER_POS)
    {
        autoMode = 2;
    }
    else
    {
        autoMode = 3;
    }

   if(tempStartPos != autoStartPosition || tempL1Bay != autoL1Bay)
   {
       modeChange = true;
   }
   // printf("modetrue\n");
   if(modeChange)
   {
        printf("\nAUTOMODE SETUP:\n");
        printf("START: %s\n", PrintMode(autoStartPosition, -1, -1, -1, -1));
        printf("LEVEL 1: %s\n", PrintMode(-1, autoL1Bay, -1, -1, -1));
        printf("         function==> %s \n", PrintMode(-1, -1, -1, -1, autoMode));       
   }

    AutoMultipliers();
}

const char* TalonXX::PrintMode(int startPos, int L1Bay, int L2Object, int L2Bay, int mode)
{
    //SIddharth - we need comments in here above each "section" of the printing options
    if(startPos != -1)
    {
        switch(startPos)
        {
            case 0:
                return("Do Nothing");
                break;

            case 1:
                return("Full Manual");
                break;
            
            case 2:
                return("Left");
                break;

            case 3:
                return("Center");
                break;

            case 4:
                return("Right");
                break;
        }
    }

    if(L1Bay != -1)
    {
        switch(L1Bay)
        {
            case 0:
                return("Panel - End Bay");
                break;
            
            case 4:
                return("Panel - Near Bay");
                break;

            case 5:
                return("Baseline");
                break;

            case 6:
                return("Panel - Rocket");
                break;

            case 7:
                return("Panel - Near Rocket");
                break;
        }
    }

    // Print function to be called for the specified mode
    if(mode != -1)
    {
        switch(mode)
        {
            case 0:
                return("DoNothing");
                break;

            case 1:
                return("FullManual");
                break;

            case 2:
                if(autoL1Bay == END_SHIP)
                {
                    return("EndHatchPanelFromCenter");
                    break;
                }         
                else
                {
                    return("BaselineFromCenter");
                    break;
                }
                break;

            case 3:
                if(autoL1Bay == END_SHIP)
                {
                    return("EndHatchPanelFromSide");
                    break;
                }
                else if(autoL1Bay == BASELINE)
                {
                    return("BaselineFromSide");
                    break;
                }
                else if(autoL1Bay == ROCKET)
                {
                    return("FarRocket");
                    break;
                }
                else if(autoL1Bay == ROCKET_NEAR)
                {
                    return("NearRocket");
                    break;
                }
                else
                {
                    return("SideHatchPanel");
                    break;
                }
                break;

            case 4:
                break;
        }
    }
}