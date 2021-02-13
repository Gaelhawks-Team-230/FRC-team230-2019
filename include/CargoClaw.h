#ifndef CARGOCLAW_H_
#define CARGOCLAW_H_

#include "Common.h"
#include <frc/VictorSP.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <frc/SmartDashboard/SmartDashboard.h>

/*
 * CargoClaw.h
 *
 *  Created on: Jan 13, 2019
 *  Author: Diya Patel
 */

class TalonXX;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)
#define INTAKE_SPIN_TIME        ((int)(0.25 * N1SEC))
#define OUTPUT_SPIN_TIME        ((int)(0.25 * N1SEC))
#define DOWN_TILT_TIME          ((int)(0.1 * N1SEC))
#define UP_TILT_TIME            ((int)(0.1 * N1SEC))

#ifdef PRACTICE_BOT
#define CALIBRATION_DELTA       (-4.5)
#else
#define CALIBRATION_DELTA       (1.0)//(3.5)
#endif
#define INTAKE_CARGO_CMD        (0.75)
#define OUTPUT_CARGO_CMD        (-0.75)
#define PARALLEL_ANGLE          (122.0 + CALIBRATION_DELTA)
#define UP_ANGLE                (27.0 + CALIBRATION_DELTA)
#define UP_INCREMENT            (TILT_UP_SPEED * LOOPTIME)
#define DOWN_INCREMENT          (TILT_DOWN_SPEED * LOOPTIME)
#define MANUAL_UP_INCREMENT     (0.99 * TILT_UP_SPEED * LOOPTIME) // positive SJ 3/13
#define MANUAL_DOWN_INCREMENT   (0.99 * TILT_DOWN_SPEED * LOOPTIME)
#define MANUAL_ANGLE_DEADBAND   (0.1)
#define TILT_OVERRIDE_CONSTANT  (0.6)
#define TILT_AT_GOAL_COUNT      (5)

#define TILT_DISTANCE_PER_PULSE (-0.095880682 * 4)


#define TILT_ERROR              (0.0)//(1.0)
#define AT_GOAL_ALLOWANCE       (0.2)
#define TILT_AT_GOAL_COUNT      (5)
#define TILT_KP                 (5.0)
#define TILT_KFF                (0.02)//(0.1)
#define TILT_KBANDWIDTH         (10.0)
#define TILT_K                  (200.0)
#define TILT_TAU                (0.05)
#define TOP_MAX                 (128.0 + CALIBRATION_DELTA)
#define BOTTOM_MIN              (0.0)
#define LIFT_UP_BOTTOM_STOP     (-10.0)
#define TOP_STOP                ( TOP_MAX - TILT_ERROR )
#define BOTTOM_STOP             ( BOTTOM_MIN + TILT_ERROR)
#define TILT_UP_SPEED           (150.0)//(150.0)
#define TILT_DOWN_SPEED         (-150.0)//(-150.0)
#define TILT_VEL_DOWN_LIMIT     (-300.0)
#define TILT_VEL_UP_LIMIT       (300.0)
#define TILT_MANUAL_UP_SPEED    (100.0)
#define TILT_MANUAL_DOWN_SPEED  (-100.0)
#define CALIBRATION_CMD_COUNT   (20)//(50)//((int)(0.1 * N1SEC))
#define TILT_INT_LOW_LIMIT      (-1 * TILT_K/TILT_KBANDWIDTH)
#define TILT_INT_HIGH_LIMIT     (TILT_K/TILT_KBANDWIDTH)

#define FULL_BACK_POS           (25.0 + CALIBRATION_DELTA)
#define DRIVE_BACK_TIME         ((int)(0.25 * (N1SEC)))
#define NINETY_DEGREES          (32.0 + CALIBRATION_DELTA)
#define FEEDER_STATION_UP       (36.0 + CALIBRATION_DELTA)//(38.0)
#ifdef PRACTICE_BOT
#define FLAT_ANGLE              (110.0 + CALIBRATION_DELTA)//(103.5)//(109.4)(118.0())
#else
#define FLAT_ANGLE              (108.5 + CALIBRATION_DELTA)//(110.0)//(113.50)//PRACTICE_BOT(105.0 + CALIBRATION_DELTA)//(103.5)//(109.4)(118.0())
#endif
#define CLIMB_FLAT_ANGLE        (117.4 + CALIBRATION_DELTA)//(109.4 + CALIBRATION_DELTA)//(118.0())
#define DRAG_ANGLE              (128.0 + CALIBRATION_DELTA)
#define PIXY_PROTECTION_ANGLE   (25.0 + CALIBRATION_DELTA)
#define FLOOR_PICKUP_UP_ANGLE   (55.0 + CALIBRATION_DELTA)
#define TILT_ADJUST_CONSTANT    (15.0)//(10.0)
#define CLIMB_TILT_ANGLE        (127.7 + CALIBRATION_DELTA)
#define LEV_TWO_CLIMB_PREP_ANGLE    (60.0)
#define LEV_TWO_CLIMB_DRIVE_ANGLE   (104.5 + CALIBRATION_DELTA)

#define TILT_ANGLE_CONSTANT_CARGO     (-0.8)
#define TILT_ANGLE_MIN_CARGO          (95.0 + CALIBRATION_DELTA)
#define TILT_TRACKING_ANGLE_MAX       (90.0 + CALIBRATION_DELTA)

class CargoClaw
{
    private:

        TalonXX *mainRobot;
        frc::VictorSP *wheels;
		frc::DigitalInput *cargoSensor;
        frc::VictorSP *tilt;
        frc::Encoder *angleEncoder;

        double wheelMotor;
        double goalAngle;
        double currentAngle;
        double tiltMotor;
        double posCmd;
        double posError;
        double velocity;
        double oldAngle;
        double velCmd;
        int atGoalCount;
        double velError;
        double velErrorInt;
        double oldVelocity;
 
       // Service Modes:
        bool isGrabbingCargo;
        bool isReleasingCargo;
        bool isManuallyMoving;
        bool isAtGoal;
        bool isManualOverride;
        bool isCalibrating;
        bool encoderReset;
        int calilbrationCount;
        double tiltShift;

        bool hasCargo;
        int releaseStage;
        int releaseLoopCount;
        bool ejectFlag;
        int tiltMultiplier;
        int testLoopCount;
        double oldPcmd;
        double bottomStopHeight;
        bool isGrabbingCargoTilt;
        double manualRotateCmd;
        double tiltAdjust;
        double tiltAdjustZero;
        double currentTiltAdjust;
       // bool tiltLimitOverride;
       // bool isAtGoal;


    public:
        CargoClaw(TalonXX* pRobot);

        //Functions

        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);

        void StartWheelSpin(void);
        void StopWheelSpin(void);
        void ReverseWheelSpin(void);
        void IntakeCargo(void);
        void EjectCargo(void);


        void OverrideManualRotate(double);
       void ManualOverrideOn(void);
       void ManualOverrideOff(void);
       void OverrideStop(void);
       bool isOverrideOn(void);
     
        double ReturnGoalAngle(void);
        void SetGoalAngle(double);
        bool AtMaxAngle(void);
        bool AtMinAngle(void);
        void ManualRotate(double);
        void StopClawTilt(void);
        double GetAngle(void);
        double AngleControlSystem(void);
        bool HaveCargo(void);        
        void WheelService(void);
        void AngleService(void);
        void FeederStationRelease(void);
        bool AtGoalAngle(void);
        void CargoAutoAngle(void);
        bool ReturnIsGrabbingCargoTilt(void);
        void TurnOffIsGrabbingCargoTilt(void);
        /*void NoTiltLimits(void);
        void OnTiltLimits(void);
        void TiltAdjust(double);*/
        void ReCalibrate(void);
        void ResetTiltAdjust(double);
        void DialTiltAdjust(double);
        bool IsDialAdjusted(void);
};
#endif /*CargoClaw_H_*/
