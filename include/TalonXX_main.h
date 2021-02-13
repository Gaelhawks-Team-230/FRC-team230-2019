
#pragma once

#ifndef SRC_TALONXX_MAIN_H_
#define SRC_TALONXX_MAIN_H_

#include <string>

#include <frc/Joystick.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/DriverStation.h>
#include <frc/PWMVictorSPX.h>
#include <frc/SampleRobot.h>
//#include <frc/drive/DifferentialDrive.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SmartDashboard.h>

//#include <frc/smartdashboard/CameraServer.h>

#include "Common.h"
#include "CargoClaw.h"
#include "Autonomous.h"
#include "DriveTrain.h"
#include "HatchHandler.h"
#include "DriveDistance.h"
#include "Climber.h"
#include "Elevator.h"
#include "LightController.h"
#ifdef USE_PIXY
#include "PixyTarget.h"
#include "PixyLine.h"
#endif

#define FEEDER_STATION_DRIVE_BACK_COUNT       ((int)(0.5 * N1SEC))//((int)(0.25 * N1SEC))
#define FEEDER_STATIOn_DRIVE_BACK_DISTANCE    (12.0)

/**
 * This is a demo program showing the use of the DifferentialDrive class. The
 * SampleRobot class is the base of a robot application that will automatically
 * call your Autonomous and OperatorControl methods at the right time as
 * controlled by the switches on the driver station or the field controls.
 */
class TalonXX : public frc::SampleRobot {
  public:
    frc::Joystick *joystick;
    frc::Joystick *gamepad;
    frc::PowerDistributionPanel *pdp;
    CargoClaw *theClaw;
    DriveTrain *drive;
    DriveDistance *distance;
    HatchHandler *hatchPanel;
    Climber *climb;
    Elevator *lift;
    LightController *lights;
#ifdef USE_PIXY_TARGET
    PixyTarget *targeter;
#endif
#ifdef USE_PIXY
    PixyLine *lineFollower;
#endif

    int dashCounter;
    int loopCount;
    int calTimer;
    double loopTime;
    double pixyTime;
    double startTime;
    double pixyEndTime;
    double curTime;
    double waitTime;
    bool loopOverTime;
    bool isBlueAlliance;
    int NumLoopsOverTime;
    bool isAuto;
    bool isPaused;
    bool firstTime;
    bool isAutoManualOverride;
    bool isJoystickCmd;
    bool isTracking;
    bool isLineDetected;
    int trackingStage;
    bool isAutoClimbing;
    bool seesTarget;
    bool isGyroOn;
    bool feederStationPrep;
#ifdef USE_VACUUM
    int feederVacLoopCount;
#endif

    double driveCmd;
    double rotateCmd;
    double strafeCmd;
    double forcedDrive;
    double forcedRotate;
    double forcedStrafe;
    int autoStage;
    int autoMode;

    int climbPrepCount;
    bool readyToClimb;
    bool isSettingClimb;
    bool climbPrepped;
    bool floorPickupPrepped;
    bool isCargoPickupPosition;
    bool dragPrepped;
    int dragDriveDelay;

    int autoStartPosition;
    int autoL1Bay;

    // AUTONOMOUS CHOOSERS
    frc::SendableChooser<int> *AutoPositionChooser;
    frc::SendableChooser<int> *LevelOneBay;

    double autoRotateMultiplier;
    double autoStrafeMultiplier;

  private:
  // Robot drive system
    int DO_NOTHING = 0;
    int FULL_MANUAL = 1;
    int LEFT_POS = 2;
    int CENTER_POS = 3;
    int RIGHT_POS = 4;
  
    int END_SHIP = 0;
    int SIDE_NEAR_SHIP = 4;
    int BASELINE = 5;
    int ROCKET = 6;
    int ROCKET_NEAR = 7;

  public: 
    TalonXX();
    void RobotInit() override;
    void Autonomous() override;
    void OperatorControl() override;
    void Test() override;
  	void InitializeAlliance(void);
  	void Disabled(void);
    static double Limit(double min, double max, double curValue);
    static double Min(double lower, double upper);
  	void RobotStartConfig(void);
  	void Omnicide(void);
  	void CommunityService(void);
    void ServiceDash();
  	void StopAll(void);
    void JoystickButtonHandler(void);
    const char* PrintMode(int, int, int, int, int);

    //Level One Functions
    void DoNothing(void);
    void FullManual(void);
    void BaselineFromCenter(void);
    void BaselineFromSide(void);
#ifdef USE_PIXY
    void EndHatchPanelFromCenter();
    void EndHatchPanelFromSide(void);
    void SideHatchPanel();
    void FarRocket(void);
    void NearRocket(void);
#endif
    void AutoMultipliers(void);
    void ModeSelection(void);
    double DoTracking(bool, bool, bool);
};

#endif