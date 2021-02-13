#ifndef ELEVATOR_H_
#define ELEVATOR_H_


#include "Common.h"

class TalonXX;

#define LIFT_DISTANCE_PER_PULSE (0.00390625)

#define PRAC_LIFT_CONSTANT    (0.5)
#define ELEVATOR_MANUAL_DEADBAND   (0.1)
#define ELEVATOR_OVERRIDE_CONSTANT  (1.0)

#define ELEVATOR_MAX_HEIGHT     (72.0)
#define ELEVATOR_MIN_HEIGHT     (0.0)
#ifdef PRACTICE_BOT
#define ELEVATOR_ALLOWANCE      (0.0)//COMPBOT(0.1)//(0.0)//(0.1)//(1.0)
#else
#define ELEVATOR_ALLOWANCE      (0.0)//3-23 3:10PM SJ(0.1)//(0.0)//(0.1)//(1.0)
#endif
#define ELEVATOR_AT_GOAL_ALLOWANCE  (0.1)
#define ELEVATOR_TOP_STOP_HEIGHT    (ELEVATOR_MAX_HEIGHT - ELEVATOR_ALLOWANCE)
#define ELEVATOR_BOTTOM_STOP_HEIGHT (ELEVATOR_MIN_HEIGHT + ELEVATOR_ALLOWANCE)

#define ELEVATOR_KP                             (5.0)
#define ELEVATOR_KFF                            (0.0)//(0.1)
#define ELEVATOR_K_BANDWIDTH                     (12.0)
#define ELEVATOR_K                            (-87.0)
#define ELEVATOR_TAU                            (0.07)
#define ELEVATOR_MIN_VELOCITY                   (-75.0)
#define ELEVATOR_MAX_VELOCITY                   (75.0)
#define ELEVATOR_MAX_VELOCITY_ERROR_INT         (fabs(ELEVATOR_K/ELEVATOR_K_BANDWIDTH))//(20.0)
#define ELEVATOR_MIN_VELOCITY_ERROR_INT         (-1.0 * ELEVATOR_MAX_VELOCITY_ERROR_INT)//(-20.0)

#define ELEVATOR_SPEED_UP                           (50.0)
#define ELEVATOR_SPEED_DOWN                         (-50.0)
#define ELEVATOR_UP_MAX_DELTA_POS                   (ELEVATOR_SPEED_UP * LOOPTIME)
#define ELEVATOR_DOWN_MAX_DELTA_POS                 (ELEVATOR_SPEED_DOWN * LOOPTIME)
#define ELEVATOR_MIN_CMD                            (-1.0)
#define ELEVATOR_MAX_CMD                            (1.0)
#define ELEVATOR_UP_MANUAL_DELTA                    (0.99 * ELEVATOR_UP_MAX_DELTA_POS)
#define ELEVATOR_DOWN_MANUAL_DELTA                  (0.99 * ELEVATOR_DOWN_MAX_DELTA_POS)

#define ELEVATOR_HOLD_CMD                           (0.2)
#define ELEVATOR_AT_GOAL_COUNT                      (8)

#define ELEVATOR_FLOOR_BUTTON_HEIGHT            (1.5)
#define ELEVATOR_CARGO_SHIP_HEIGHT              (38.2)
#define ELEVATOR_PICKUP_BUTTON_HEIGHT           (1.5)
#define ELEVATOR_LOW_BUTTON_HEIGHT              (0.8)//(ELEVATOR_BOTTOM_STOP_HEIGHT) //(3.5) //was 15
#define ELEVATOR_MIDDLE_BUTTON_HEIGHT           (28.18)//(30.2)
#define ELEVATOR_TOP_BUTTON_HEIGHT              (56.46)//(58.0)
#define ELEVATOR_CARGO_LOW_BUTTON               (24.11)//(21.5)
#define ELEVATOR_CARGO_MIDDLE_BUTTON            (49.0)//(50.0)
#define ELEVATOR_CARGO_HIGH_BUTTON              (72.0)

#define TILT_CHECK_HEIGHT_LOW                   (4.0)//(6.0)
#define TILT_CHECK_HEIGHT_HIGH                  (20.0)

#define ELEVATOR_HEIGHT_FEEDER_STATION          (8.0)//(5.0)
#define ELEVATOR_CLIMB_START_HEIGHT             (21.19)
#define ELEVATOR_CLIMB_TWO_START_HEIGHT         (6.19)//(7.19)//(8.19)
#define ELEVATOR_UP_MANUAL_MOVE                 (-0.2)
#define FLOOR_PICKUP_HEIGHT                     (3.5)
#define CARGO_PICKUP_HEIGHT                     (5.0)
#define CARGO_TILT_BOTTOM_LIMIT                 (40.0)
#define ELEVATOR_DRAG_HEIGHT                    (2.0)
//#define FEEDER_STATION_HEIGHT                   (0.8)

#define ELEVATOR_TRACKING_MAX_HEIGHT            (3.5)
#define ELEVATOR_CARGO_SHIP_HATCH_HEIGHT        (3.0)

class Elevator
{
    private:
        frc::Encoder *encoder;
		frc::VictorSP *elevator1;
        //frc::VictorSP *elevator2;

        TalonXX *mainRobot;

        float motorCmd;
        double currentPosition;
		double oldPosition;
		double positionError;
		double curVelocity;
		bool atGoal;
		double positionCmd;
		double velocityCmd;
		double velocityError;
		double velocityErrorInt;
		double goalPosition;
		bool isManualMove;
		double oldPCmd;
        bool isManualOverride;
        int atGoalCount;
        double holdGoalPosition;
        bool isCalibrating;
        bool encoderReset;

    public:
        Elevator(TalonXX* pRobot);
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        double GetHeight(void);
        void UpdateDash(void);
        void Service(void);
        void ManualMove(double);
        void OverrideManualMove(double);
        void OverrideHold(void);
        void TurnOnManualOverride(void);
        void TurnOffManualOverride(void);
        bool isOverrideOn(void);
        void GiveGoalPosition(double);
        void ControlMove(void);
        double GetGoalPosition(void);
        bool AtFloor(void);
        bool AtMax(void);
        bool AtGoalPosition(void);
        void LiftStop(void);
        bool ReturnEncoderReset(void);
};
#endif /*Elevator_H_*/
