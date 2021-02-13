
#include <iostream>

#include <frc/Timer.h>
#include "TalonXX_main.h"

#include "ctre/Phoenix.h"

//TalonSRX srx = {0};

TalonXX::TalonXX() 
{
	// Note SmartDashboard is not initialized here, wait until RobotInit() to make
	// SmartDashboard calls
	joystick = new frc::Joystick(0);
	gamepad = new frc::Joystick(1);
	pdp = new frc::PowerDistributionPanel();
	
	theClaw = new CargoClaw(this);
	hatchPanel = new HatchHandler(this);
	distance = new DriveDistance(this);
	drive = new DriveTrain(this);
	climb = new Climber(this);
	lift = new Elevator(this);
	lights = new LightController(this);
#ifdef USE_PIXY_TARGET
	targeter = new PixyTarget(this);
#endif
#ifdef USE_PIXY
	lineFollower = new PixyLine(this);
#endif
	
	AutoPositionChooser = new frc::SendableChooser<int>;
	LevelOneBay = new frc::SendableChooser<int>;

	autoRotateMultiplier = 1.0;
	autoStrafeMultiplier = 1.0;

	dashCounter = 0;
	loopTime = LOOPTIME;
	pixyTime = PIXYTIME;
	startTime = 0.0;
	pixyEndTime = 0.0;
	curTime = 0.0;
	waitTime = 0.0;
	loopOverTime = false;
	NumLoopsOverTime = 0;
	loopCount = 0;
	calTimer = 0;
	dragDriveDelay = 0;
	isBlueAlliance = false;
	firstTime = true;
	isAuto = false;
	isPaused = false;
	isAutoManualOverride = false;
	isJoystickCmd = false;
	isTracking = false;
	isLineDetected = false;
	trackingStage = 0;
	isAutoClimbing = false;
	isSettingClimb = false;
	readyToClimb = false;
	climbPrepCount = 0;

	seesTarget = false;
	isCargoPickupPosition = false;
	feederStationPrep = false;

	autoStartPosition = 0;
	autoL1Bay = 0;

	driveCmd = 0.0;
	rotateCmd = 0.0;
	strafeCmd = 0.0;
	forcedDrive = 0.0;
	forcedRotate = 0.0;
	forcedStrafe = 0.0;
	autoStage = 0;
	autoMode = 0;
#ifdef USE_GYRO
		isGyroOn = true;
#else
		isGyroOn = false;
#endif
#ifdef USE_VACUUM
	feederVacLoopCount = 0;
#endif
	climbPrepped = false;
	floorPickupPrepped = false;
}

void TalonXX::RobotInit() 
{
#ifdef USE_USB_CAMERA
	frc::CameraServer::GetInstance()->StartAutomaticCapture();
#endif

	AutoPositionChooser->AddOption("Left", LEFT_POS);
	AutoPositionChooser->SetDefaultOption("Center", CENTER_POS);
	AutoPositionChooser->AddOption("Right", RIGHT_POS);
	AutoPositionChooser->AddOption("Do Nothing", DO_NOTHING);
	AutoPositionChooser->AddOption("Full Manual", FULL_MANUAL);
	frc::SmartDashboard::PutData("Positions: ", AutoPositionChooser);

	LevelOneBay->SetDefaultOption("L1EndLeft", END_SHIP);
	LevelOneBay->AddOption("L1SideNear", SIDE_NEAR_SHIP);
	LevelOneBay->AddOption("Do Nothing", BASELINE);
	LevelOneBay->AddOption("RocketFar", ROCKET);
	LevelOneBay->AddOption("RocketNear", ROCKET_NEAR);
	frc::SmartDashboard::PutData("L1Bay: ", LevelOneBay);

  	ServiceDash();
}

void TalonXX::Disabled()
{
	while(IsDisabled())
	{
		ModeSelection();
		frc::Wait(loopTime);
		isAutoManualOverride = false;
	}
}

void TalonXX::InitializeAlliance()
{
	if(frc::DriverStation::GetInstance().GetAlliance() == frc::DriverStation::kBlue)
		isBlueAlliance = true;
	else
		isBlueAlliance = false;
}

void TalonXX::Autonomous() 
{
	ModeSelection();

	double initialStartTime;
	isAuto = true;
	InitializeAlliance();

	if(firstTime)
	{
		theClaw->ResetTiltAdjust(joystick->GetRawAxis(TILT_STOP_LIMIT));
	}

	lift->LocalReset();
	theClaw->LocalReset();
	climb->LocalReset();
	drive->LocalReset();
#ifdef USE_VACUUM
	hatchPanel->HoldVacuum();
#endif

	initialStartTime = frc::GetTime();
	startTime = initialStartTime - loopTime;
  	while (IsAutonomous() && IsEnabled()) 
	{
		startTime += loopTime;
	#ifdef CHECK_LOOPTIMING
		loopCounter++;
	#endif
		forcedDrive = 0.0; forcedStrafe = 0.0; forcedRotate = 0.0;
		if(isTracking)
		{
#ifdef USE_PIXY_TARGET
            targeter->AnalyzeTarget();
			if(targeter->GetNumTargets() > 0)
			{
				seesTarget = true;
			}
			else
			{
				seesTarget = false;
			}
#else
			seesTarget = false;
#endif
#ifdef USE_PIXY    
			isLineDetected = lineFollower->LineDetected();
#else
			isLineDetected = false;
#endif
		}
		else
		{
			seesTarget = false;
#ifdef USE_PIXY_TARGET
			targeter->ResetTargets();
#endif
		}

		pixyEndTime = frc::GetTime();
		waitTime = pixyTime - (pixyEndTime - startTime);

		if (waitTime < 0.0)
		{
			printf("WARNING! PIXY IS OVERTIME by %f seconds\n", waitTime*-1);
		}
		else
		{
			frc::Wait(waitTime);
		}
		theClaw->DialTiltAdjust(joystick->GetRawAxis(TILT_STOP_LIMIT));
		if(calTimer > ONE_SEC)
		{
			if(joystick->GetRawButton(OVERRIDE_BUTTON_RIGHT))
			{
				isAutoManualOverride = true;
				isPaused = false;
				autoMode = 1;
				isJoystickCmd = true;
			}
			if(joystick->GetRawButton(AUTO_PAUSE_BUTTON)) 
			{
				isPaused = true;
			}
			else
			{
				drive->GyroOn();
				isPaused = false;
				isJoystickCmd = false;
				switch(autoMode)
				{
					case 0:
						DoNothing();
						break;

					case 1:
						FullManual();
						break;

#ifdef USE_PIXY
					case 2:
						if(autoL1Bay == END_SHIP)
						{
							EndHatchPanelFromCenter();
						}
						else
						{
							BaselineFromCenter();
						}
						break;

					case 3:
						if(autoL1Bay == END_SHIP)
						{
							EndHatchPanelFromSide();
						}
						else if(autoL1Bay == BASELINE)
						{
							BaselineFromSide();
						}
						else if(autoL1Bay == ROCKET)
						{
							FarRocket();
						}
						else if(autoL1Bay == ROCKET_NEAR)
						{
							NearRocket();
						}
						else
						{
							SideHatchPanel();
						}
						break;

					case 4:
						break;
#endif
				}
			}
		}
		else
		{
			calTimer++;
		}
		
		

		CommunityService();
		ServiceDash();
		drive->DriveControl(driveCmd, strafeCmd, rotateCmd, forcedDrive, forcedStrafe, forcedRotate, isJoystickCmd, isAuto, isPaused);

		curTime = frc::GetTime();
		waitTime = loopTime - (curTime - startTime);
		if (waitTime < 0.0)
		{
			printf("WARNING! LOOP IS OVERTIME by %f seconds\n", waitTime*-1);
			loopOverTime = true;
			NumLoopsOverTime++;
			
		}
		else
		{
			frc::Wait(waitTime);
			loopOverTime = false;

	#ifdef CHECK_LOOPTIMING
			endTime = GetTime();
			totalLoopTime = endTime - startTime;
			//printf("startTime: %f  curTime : %f  endTime: %f	Wait Time: %f  This Loop Time: %f  Total Delta Time: %f [avg: %f] \n",
			//		startTime, curTime, endTime, waitTime, totalLoopTime, endTime-initialStartTime, (endTime-initialStartTime)/loopCounter);
			printf("Wait Time: %f  \n",	   waitTime);
	#endif
		}
	}
	Omnicide();
}

void TalonXX::OperatorControl()
{
	int loopCounter=0;
	double initialStartTime;
	isAuto = false;
	InitializeAlliance();
	
	if(firstTime)
	{
		theClaw->ResetTiltAdjust(joystick->GetRawAxis(TILT_STOP_LIMIT));
	}

	lift->LocalReset();
	theClaw->LocalReset();
	climb->LocalReset();
	drive->LocalReset();
	initialStartTime = frc::GetTime();
	startTime = initialStartTime - loopTime;

	while (IsOperatorControl() && IsEnabled()) 
	{
		startTime += loopTime;
	#ifdef CHECK_LOOPTIMING
		loopCounter++;
	#endif

		isJoystickCmd = true;
		if(isTracking)
		{
#ifdef USE_PIXY_TARGET
            targeter->AnalyzeTarget();
#else
			seesTarget = false;
#endif
#ifdef USE_PIXY
			isLineDetected = lineFollower->LineDetected();
#endif
		}
		else
		{
			seesTarget = false;
#ifdef USE_PIXY_TARGET
			targeter->ResetTargets();
#endif
		}
		theClaw->DialTiltAdjust(joystick->GetRawAxis(TILT_STOP_LIMIT));
		JoystickButtonHandler();


		pixyEndTime = frc::GetTime();
		waitTime = pixyTime - (pixyEndTime - startTime);

		if (waitTime < 0.0)
		{
			printf("WARNING! PIXY IS OVERTIME by %f seconds\n", waitTime*-1);
		}
		else
		{
			frc::Wait(waitTime);
		}
	
		CommunityService();
		ServiceDash();
		drive->DriveControl(driveCmd, strafeCmd, rotateCmd, forcedDrive, forcedStrafe, forcedRotate, isJoystickCmd, isAuto, isPaused);

		curTime = frc::GetTime();
		waitTime = loopTime - (curTime - startTime);
		if (waitTime < 0.0)
		{
			printf("WARNING! LOOP IS OVERTIME by %f seconds\n", waitTime*-1);
			loopOverTime = true;
			NumLoopsOverTime++;
			
		}
		else
		{
			frc::Wait(waitTime);
			loopOverTime = false;

	#ifdef CHECK_LOOPTIMING
			endTime = GetTime();
			totalLoopTime = endTime - startTime;
			//printf("startTime: %f  curTime : %f  endTime: %f	Wait Time: %f  This Loop Time: %f  Total Delta Time: %f [avg: %f] \n",
			//		startTime, curTime, endTime, waitTime, totalLoopTime, endTime-initialStartTime, (endTime-initialStartTime)/loopCounter);
			printf("Wait Time: %f  \n",	   waitTime);
	#endif
		}
	}
  	Omnicide();

}

void TalonXX::StopAll()
{
	loopCount = 0;
	autoStage = 0;
	driveCmd = 0.0; rotateCmd = 0.0; strafeCmd = 0.0;
	theClaw->StopAll();
	drive->StopAll();
	distance->StopAll();
	hatchPanel->StopAll();
	climb->StopAll();
	lift->StopAll();
	lights->StopAll();
#ifdef USE_PIXY_TARGET
	targeter->StopAll();
#endif
#ifdef USE_PIXY
	lineFollower->StopAll();
#endif
}

void TalonXX::Test() 
{

}

double TalonXX::Limit(double min, double max, double curValue)
{
	if (curValue > max)
		return max;
	if (curValue < min)
		return min;
	return curValue;
}

double TalonXX::Min(double lower, double upper)
{
	if(lower <= upper)
	{
		return lower;
	}
	else
	{
		return upper;
	}
}

void TalonXX::RobotStartConfig()
{
	theClaw->StartingConfig();
	drive->StartingConfig();
	distance->StartingConfig();
	hatchPanel->StartingConfig();
	climb->StartingConfig();
	lift->StartingConfig();
	lights->StartingConfig();
#ifdef USE_PIXY_TARGET
	targeter->StartingConfig();
#endif
#ifdef USE_PIXY
	lineFollower->StartingConfig();
#endif
}

void TalonXX::Omnicide()
{
	isPaused = false;
    calTimer = 0;
	isSettingClimb = false;
	readyToClimb = false;
	climbPrepCount = 0;
	StopAll();
}

void TalonXX::CommunityService()
{
	theClaw->Service();
	drive->Service();
	distance->Service();
	hatchPanel->Service();
	climb->Service();
	lift->Service();
	lights->Service();
#ifdef USE_PIXY_TARGET
	targeter->Service();
#endif
#ifdef USE_PIXY
	lineFollower->Service();
#endif
}

void TalonXX::ServiceDash()
{
	if(dashCounter == 20)
	{
		theClaw->UpdateDash();
		drive->UpdateDash();
		distance->UpdateDash();
		hatchPanel->UpdateDash();
		climb->UpdateDash();
		lift->UpdateDash();
#ifdef USE_PIXY_TARGET
		targeter->UpdateDash();
#endif
#ifdef USE_PIXY
		lineFollower->UpdateDash();
#endif
		frc::SmartDashboard::PutBoolean("isPaused", isPaused);
		frc::SmartDashboard::PutBoolean("isAutoManualOverride", isAutoManualOverride);
		frc::SmartDashboard::PutBoolean("readyToClimb", readyToClimb);
	}
	else
	{
		dashCounter++;
	}
}

double TalonXX::DoTracking(bool isRotate, bool leftTarget, bool isTargetting)
{
	int numTargets = 0;
	double targetCenter = 0.0;
	double trackingCmd = 0.0;

    isTracking = true;
           
#ifdef USE_PIXY_TARGET
    if(isTargetting)
	{
		if(targeter->GetNumTargets() > 0)
		{
			numTargets = targeter->GetNumTargets();
			if(numTargets > 1)
			{
				if(leftTarget)
				{
					targetCenter = targeter->GetCompositeTargetX(1);
				}
				else
				{
					targetCenter = targeter->GetCompositeTargetX(numTargets);
				}
				
			}
			else
			{
				targetCenter = targeter->GetCompositeTargetX(1);
			}
			
			
			//printf("Pixy targetting %d\n", targeter->GetCompositeTargetX(1));
			if(isRotate)
			{
				trackingCmd = (TARGET_CENTER_VALUE - targetCenter) * TARGET_ROTATE_MULTIPLIER;
			}
			else
			{
				trackingCmd = (TARGET_CENTER_VALUE - targetCenter) * TARGET_STRAFE_MULTIPLIER;
			}
		}
	}
#endif
#ifdef USE_PIXY
    if(isLineDetected)
    {
        //printf("isLineDetected %f\n", lineFollower->LineLocation());
        if(isRotate)
		{
			trackingCmd = (LINE_CENTER_VALUE - lineFollower->LineLocation()) * LINE_ROTATE_MULTIPLIER;                
		}
		else
		{
			trackingCmd = (LINE_CENTER_VALUE - lineFollower->LineLocation()) * LINE_STRAFE_MULTIPLIER;                
		}
    }
#endif
	return (trackingCmd);
}

void TalonXX::JoystickButtonHandler()
{
	forcedDrive = 0.0; forcedRotate = 0.0; forcedStrafe = 0.0;
	if(lift->GetHeight() < ELEVATOR_TRACKING_MAX_HEIGHT && theClaw->GetAngle() < TILT_TRACKING_ANGLE_MAX)
	{
		if(joystick->GetRawAxis(TRACKING_AXIS) > 0.9 )
		{
			isTracking = true;
			
	#ifdef STRAFE_TRACKING
			strafeCmd = DoTracking(false, true, isAuto); 
			if (strafeCmd == 0.0)
			{
				strafeCmd = joystick->GetRawAxis(STRAFE_AXIS);
			}
			driveCmd = (joystick->GetRawAxis(SPEED_AXIS) * -1.0);
			rotateCmd = joystick->GetRawAxis(ROTATE_AXIS);
	#else
			rotateCmd = DoTracking(true, true, isAuto); 
			if (rotateCmd == 0.0)
			{
				rotateCmd = joystick->GetRawAxis(ROTATE_AXIS);
			}
			driveCmd = (joystick->GetRawAxis(SPEED_AXIS) * -1.0);
			strafeCmd = joystick->GetRawAxis(STRAFE_AXIS);
	#endif
		}
		else if(joystick->GetRawButton(TRACKING_BUTTON))
		{
			isTracking = true;
	#ifdef STRAFE_TRACKING
			rotateCmd = DoTracking(true, true, isAuto); 
			if (rotateCmd == 0.0)
			{
				rotateCmd = joystick->GetRawAxis(ROTATE_AXIS);
			}
			driveCmd = (joystick->GetRawAxis(SPEED_AXIS) * -1.0);
			strafeCmd = joystick->GetRawAxis(STRAFE_AXIS);
	#else
			strafeCmd = DoTracking(false, true, isAuto); 
			if (strafeCmd == 0.0)
			{
				strafeCmd = joystick->GetRawAxis(STRAFE_AXIS);
			}
			driveCmd = (joystick->GetRawAxis(SPEED_AXIS) * -1.0);
			rotateCmd = joystick->GetRawAxis(ROTATE_AXIS);
	#endif
		}
		else
		{
			isTracking = false;
			isJoystickCmd = true;
			driveCmd = (joystick->GetRawAxis(SPEED_AXIS) * -1.0);
			strafeCmd = joystick->GetRawAxis(STRAFE_AXIS);
			rotateCmd = joystick->GetRawAxis(ROTATE_AXIS);
		}
	}
	else
	{
		isTracking = false;
		isJoystickCmd = true;
		driveCmd = (joystick->GetRawAxis(SPEED_AXIS) * -1.0);
		strafeCmd = joystick->GetRawAxis(STRAFE_AXIS);
		rotateCmd = joystick->GetRawAxis(ROTATE_AXIS);
	}

	if(joystick->GetRawButton(CLIMB_CHECK_BUTTON) && joystick->GetRawButton(CLIMB_LEV3_BUTTON))
	{
		if(readyToClimb)
		{
			climb->TurnOnAutoClimb();
		}
	}
	else if(joystick->GetRawButton(CLIMB_TWO_PREP) && joystick->GetRawButton(CLIMB_LEV3_BUTTON))
	{
		if(readyToClimb)
		{
			climb->TurnOnLevTwoClimb();
		}
	}
	else if(joystick->GetRawButton(CLIMB_CHECK_BUTTON))
	{
#ifdef USE_VACUUM
		hatchPanel->ReleaseVacuum();
#endif
		climbPrepCount++;
		if(climbPrepCount > CLIMB_PREP_TIME)
		{
			readyToClimb = true;
			isSettingClimb = false;
		}
		else
		{
			isSettingClimb = true;
		}
		
		isGyroOn = false;
		if(!climbPrepped)
		{
			theClaw->SetGoalAngle(NINETY_DEGREES);
			lift->GiveGoalPosition(ELEVATOR_CLIMB_START_HEIGHT);
			if(lift->AtGoalPosition())
			{
				theClaw->SetGoalAngle(CLIMB_FLAT_ANGLE);
				climbPrepped = true;
			}
		}
		forcedDrive = 0.15; forcedStrafe = 0.0; forcedRotate = 0.0;
	}
	else if(joystick->GetRawButton(CLIMB_TWO_PREP))
	{
#ifdef USE_VACUUM
		hatchPanel->ReleaseVacuum();
#endif
		climbPrepCount++;
		if(climbPrepCount > CLIMB_TWO_PREP_TIME)
		{
			readyToClimb = true;
			isSettingClimb = false;
		}
		else
		{
			isSettingClimb = true;
		}
		
		isGyroOn = false;
		if(!climbPrepped)
		{
			theClaw->SetGoalAngle(LEV_TWO_CLIMB_PREP_ANGLE);
			climbPrepped = true;
		}
		forcedDrive = 0.15; forcedStrafe = 0.0; forcedRotate = 0.0;
	}
	else
	{
		climbPrepped = false;
		isSettingClimb = false;
		readyToClimb = false;
		climbPrepCount = 0;
		climb->TurnOffAutoClimb();
		climb->TurnOffLevTwoClimb();
		if(joystick->GetRawButton(ENABLE_GYRO_BUTTON))
		{
			isGyroOn = true;
		}
		else
		{
			isGyroOn = false;
		}
	}

	
	
	if(joystick->GetRawButton(OVERRIDE_MAIN) && joystick->GetRawButton(OVERRIDE_CLIMBER) && joystick->GetRawButton(OVERRIDE_ELEVATOR) && joystick->GetRawButton(OVERRIDE_PIVOT))
	{
		RobotStartConfig();
	}
	else
	{
		if(joystick->GetRawButton(OVERRIDE_MAIN) && joystick->GetRawButton(OVERRIDE_CLIMBER))
		{
			climb->TurnOnOverride();
		}
		else if(joystick->GetRawButton(OVERRIDE_CLIMBER))
		{
			climb->TurnOffOverride();
		}

		if(joystick->GetRawButton(OVERRIDE_MAIN) && joystick->GetRawButton(OVERRIDE_ELEVATOR))
		{
			lift->TurnOnManualOverride();
		}
		else if(joystick->GetRawButton(OVERRIDE_ELEVATOR))
		{
			lift->TurnOffManualOverride();
		}

		if(joystick->GetRawButton(OVERRIDE_MAIN) && joystick->GetRawButton(OVERRIDE_PIVOT))
		{
			theClaw->ManualOverrideOn();
		}
		else if(joystick->GetRawButton(OVERRIDE_PIVOT))
		{
			theClaw->ManualOverrideOff();
		}
	}

	if(joystick->GetRawButton(DRAG_BUTTON))
	{
		if(!dragPrepped)
		{
			theClaw->SetGoalAngle(DRAG_ANGLE);
			lift->GiveGoalPosition(ELEVATOR_DRAG_HEIGHT);
			dragPrepped = true;
			dragDriveDelay = 0;
		}
		if(dragDriveDelay > DRAG_DELAY)
		{
			forcedDrive = -0.4;
		}
		else
		{
			dragDriveDelay++;
		}
	}
	else
	{
		if(dragPrepped)
		{
			hatchPanel->ReleaseHatch();
			lift->GiveGoalPosition(ELEVATOR_BOTTOM_STOP_HEIGHT);
			theClaw->SetGoalAngle(FLAT_ANGLE);
			//hatchPanel->TurnOnFloorPickup();
			dragPrepped = false;
			dragDriveDelay = 0;
		}
	}
	
	if(!lift->isOverrideOn())
	{
		if(gamepad->GetRawButton(CARGO_MOD))
		{
			if(gamepad->GetRawButton(HATCH_PICKUP_BUTTON))
			{
				lift->GiveGoalPosition(ELEVATOR_CARGO_SHIP_HEIGHT);
			}
			if(gamepad->GetRawButton(ELEVAOTR_LOW_BUTTON))
			{
				lift->GiveGoalPosition(ELEVATOR_CARGO_LOW_BUTTON);
			}
			if(gamepad->GetRawButton(ELEVATOR_MIDDLE_BUTTON))
			{
				lift->GiveGoalPosition(ELEVATOR_CARGO_MIDDLE_BUTTON);
			}
			if(gamepad->GetRawButton(ELEVATOR_TOP_BUTTON))
			{
				lift->GiveGoalPosition(ELEVATOR_CARGO_HIGH_BUTTON);
			}		
		}
		else
		{
			if(gamepad->GetRawButton(HATCH_PICKUP_BUTTON))
			{
				//lift->GiveGoalPosition(ELEVATOR_PICKUP_BUTTON_HEIGHT);
				//hatchPanel->TurnOnFloorPickup();
				if(!floorPickupPrepped)
				{
					theClaw->SetGoalAngle(FLAT_ANGLE);
					lift->GiveGoalPosition(ELEVATOR_BOTTOM_STOP_HEIGHT);
					floorPickupPrepped = true;
				}
				theClaw->TurnOffIsGrabbingCargoTilt();
				
				isCargoPickupPosition = false;

			}
			else
			{
				if(floorPickupPrepped)
				{
					hatchPanel->TurnOnFloorPickup();
					floorPickupPrepped = false;
				}
			}
			
			if(gamepad->GetRawButton(ELEVAOTR_LOW_BUTTON))
			{
				lift->GiveGoalPosition(ELEVATOR_LOW_BUTTON_HEIGHT);
			}
			if(gamepad->GetRawButton(ELEVATOR_MIDDLE_BUTTON))
			{
				lift->GiveGoalPosition(ELEVATOR_MIDDLE_BUTTON_HEIGHT);
			}
			if(gamepad->GetRawButton(ELEVATOR_TOP_BUTTON))
			{
				lift->GiveGoalPosition(ELEVATOR_TOP_BUTTON_HEIGHT);
			}		
		}	
	}
	


	if(gamepad->GetRawButton(WHEELS_OUT_BUTTON))
	{
		theClaw->EjectCargo();
		isCargoPickupPosition = false;
	}
	else if(gamepad->GetRawButton(WHEELS_IN_BUTTON))
	{
		theClaw->IntakeCargo();
		if(!isCargoPickupPosition)
		{
			lift->GiveGoalPosition(CARGO_PICKUP_HEIGHT);
			theClaw->SetGoalAngle(TOP_STOP);
			isCargoPickupPosition = true;
			floorPickupPrepped = false;

		}
	}
	else
	{
		theClaw->StopWheelSpin();
	}
	
	if(gamepad->GetRawButton(CARGO_MOD) && gamepad->GetRawButton(PIVOT_UP_BUTTON) && gamepad->GetRawButton(PIVOT_DOWN_BUTTON))
	{
		theClaw->ReCalibrate();
	}
	else
	{
		if(!theClaw->isOverrideOn())
		{
			if(gamepad->GetRawButton(PIVOT_UP_BUTTON))
			{
				theClaw->SetGoalAngle(FEEDER_STATION_UP);
			}
			if(gamepad->GetRawButton(PIVOT_DOWN_BUTTON))
			{
				theClaw->SetGoalAngle(FLAT_ANGLE);
			}
		}
	}

	if(lift->isOverrideOn())
	{
		lift->OverrideManualMove(gamepad->GetRawAxis(ELEVATOR_AXIS));
	}
	else
	{
		lift->ManualMove(gamepad->GetRawAxis(ELEVATOR_AXIS));
	}

	if(theClaw->isOverrideOn())
	{
		theClaw->OverrideManualRotate(gamepad->GetRawAxis(TILT_AXIS));
		
	}
	else
	{
		double tiltAxisReading;
		tiltAxisReading = gamepad->GetRawAxis(TILT_AXIS);
		if(fabs(tiltAxisReading) <= MANUAL_ANGLE_DEADBAND)
		{
			tiltAxisReading = 0.0;
		}
		else if (tiltAxisReading >= 0.0)
		{
			tiltAxisReading = (tiltAxisReading - MANUAL_ANGLE_DEADBAND)/(1.0 - MANUAL_ANGLE_DEADBAND);
		}
		else
		{
			tiltAxisReading = (tiltAxisReading + MANUAL_ANGLE_DEADBAND)/(1.0 - MANUAL_ANGLE_DEADBAND);
		}
		tiltAxisReading = tiltAxisReading * (1.0 + 0.5 * 0.5 * (fabs(tiltAxisReading) - 1.0));
		tiltAxisReading = tiltAxisReading*TILT_MANUAL_UP_SPEED/TILT_UP_SPEED;
		theClaw->ManualRotate(tiltAxisReading);
	}


	/*if(climb->isOverrideOn())
	{
		climb->OverrideManualMove(gamepad->GetRawAxis(TILT_AXIS));
	}
	else
	{
		climb->ManualMove(gamepad->GetRawAxis(TILT_AXIS));
	}*/
	

	if(gamepad->GetPOV(0) == (DPAD_CAM_RELEASE))
	{
		hatchPanel->ReleaseHatch();
	}
	
	/*if(gamepad->GetPOV(0) == (DPAD_CLIMB_RETRACT))
	{
		climb->ManualMove(-1.0);
	}*/

	if(!climb->isOverrideOn())
	{
		if(gamepad->GetPOV(0) == (DPAD_CLIMBER_MANUAL_UP))
		{
			climb->ManualMove(1.0);
			lift->ManualMove(0.11);
		}
		if(gamepad->GetPOV(0) == (DPAD_CLIMBER_MANUAL_DOWN))
		{
			climb->ManualMove(-1.0);
			lift->ManualMove(-0.11);
		}
		
	}
	else
	{
		if(gamepad->GetPOV(0) == (DPAD_CLIMBER_MANUAL_UP))
		{
			climb->OverrideManualMove(0.6);
		}
		else if(gamepad->GetPOV(0) == (DPAD_CLIMBER_MANUAL_DOWN))
		{
			climb->OverrideManualMove(-0.6);
		}
		else
		{
			climb->OverrideManualMove(0.0);
			//climb->OverrideManualMove(gamepad->GetRawAxis(0));

		}
	}

	if(gamepad->GetPOV(0) == (DPAD_STOP_MECHANISMS))
	{
		if(!lift->isOverrideOn())
		{
			lift->LiftStop();
		}
		else
		{
			lift->OverrideHold();
		}
		if(!climb->isOverrideOn())
		{
			climb->ClimbStop();
		}
		if(!theClaw->isOverrideOn())
		{
			theClaw->StopClawTilt();
		}
		else
		{
			theClaw->OverrideStop();
		}
		hatchPanel->StopReleaseHatch();
		theClaw->StopWheelSpin();
	}

	if(gamepad->GetRawButton(FEEDER_STATION_BUTTON))
	//if(gamepad->GetPOV(0) == (DPAD_FEEDER_STATION))
	{
		isGyroOn = false;

		//isTracking = true;
		if(!feederStationPrep)
		{
#ifdef USE_VACUUM
			hatchPanel->StartVacuum();
			feederVacLoopCount = 0;
#endif
			theClaw->SetGoalAngle(FEEDER_STATION_UP);
			lift->GiveGoalPosition(ELEVATOR_BOTTOM_STOP_HEIGHT);
			feederStationPrep = true;
			// Need to have this to prepare it to be able to restart if button is pushed again...
			// before it has completed the release operation after a failed attempt to grab it a hatch
			hatchPanel->TurnOffFeederStationRelease();
		}
		forcedDrive = 0.35; forcedStrafe = 0.0; forcedRotate = 0.0;
	}
	else
	{

		if(feederStationPrep)
		{
			hatchPanel->TurnOnFeederStationRelease();
			isGyroOn = true;
#ifdef USE_VACUUM
			feederVacLoopCount++;
			if(feederVacLoopCount > ONE_SEC)
			{
				hatchPanel->HoldVacuum();
				feederStationPrep = false;
			}
#endif
		}
	}
	
	
//HARTFORD SJ	theClaw->DialTiltAdjust(joystick->GetRawAxis(TILT_STOP_LIMIT));

	if(isGyroOn)
	{
		drive->GyroOn();
	}
	else
	{
		drive->GyroOff();
	}
	
	/*if(joystick->GetRawAxis(TILT_STOP_LIMIT) > 0.9)
	{
		theClaw->ReCalibrate();
	}*/
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<TalonXX>(); }
#endif
