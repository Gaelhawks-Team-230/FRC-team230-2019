#ifndef CLIMBER_H_
#define CLIMBER_H_

#include "Common.h"

class TalonXX;

#define CLIMBER_DISTANCE_PER_PULSE (-0.0183333)

#define CLIMBER_OVERRIDE_CONSTANT  (1.0)
#define CLIMBER_MANUAL_DEADBAND   (0.1)

#define CLIMBER_MAX_HEIGHT     (25.0)//(24.0)(25.0)//(27.0)
#define CLIMBER_MIN_HEIGHT     (0.0)
#define CLIMB_FLAT_HEIGHT       (19.0)
#define CLIMB_CLEARANCE_DRIVE_HEIGHT    (1.0)
#ifdef PRACTICE_BOT
#define CLIMBER_LEV_TWO_CLIMB   (6.5)//(19.0)//(17.0)//(15.0)
#else
#define CLIMBER_LEV_TWO_CLIMB   (6.5)
#endif
#define CLIMBER_ALLOWANCE      (0.1)
#define CLIMBER_TOP_STOP_HEIGHT    (CLIMBER_MAX_HEIGHT - CLIMBER_ALLOWANCE)
#define CLIMBER_BOTTOM_STOP_HEIGHT (CLIMBER_MIN_HEIGHT + CLIMBER_ALLOWANCE)

#define CLIMBER_KP                             (5.0)
#define CLIMBER_KFF                            (0.0)//(0.1)
#define CLIMBER_K_BANDWIDTH                     (12.0)
#define CLIMBER_K                              (18.0)//(10.0)
#define CLIMBER_TAU                            (0.07)//(0.05)
#define CLIMBER_MIN_VELOCITY                   (-20.0)//(-10.0) //(-1 * CLIMBER_K/CLIMBER_K_BANDWIDTH)
#define CLIMBER_MAX_VELOCITY                   (20.0)//(10.0)// (CLIMBER_K/CLIMBER_K_BANDWIDTH)
#define CLIMBER_MIN_VELOCITY_ERROR_INT         (-1 * CLIMBER_K/CLIMBER_K_BANDWIDTH)
#define CLIMBER_MAX_VELOCITY_ERROR_INT         (CLIMBER_K/CLIMBER_K_BANDWIDTH)

#define CLIMBER_SPEED_UP                           (5.0)
#define CLIMBER_SPEED_DOWN                         (-5.0)

#define CLIMBER_MIN_CMD                            (-1.0)//(-0.5)
#define CLIMBER_MAX_CMD                            (1.0)//(0.5)
#define CLIMBER_UP_MANUAL_DELTA                    (0.99 * CLIMBER_UP_MAX_DELTA_POS)
#define CLIMBER_DOWN_MANUAL_DELTA                  (0.99 * CLIMBER_DOWN_MAX_DELTA_POS)
#define CLIMBER_UP_MAX_DELTA_POS                   (CLIMBER_SPEED_UP * LOOPTIME)
#define CLIMBER_DOWN_MAX_DELTA_POS                 (CLIMBER_SPEED_DOWN * LOOPTIME)
#define CLIMB_DRIVE_CMD                            (0.2)
#define CLIMB_POST_DELAY                        ((int)(1.0 * N1SEC))

#define CLIMB_LEV3_EXTEND                       (21.0)
#define DRIVE_ONTO_PLATFORM_INITIAL                     (10.0)
#define COMPLETE_DRIVE_ONTO_PLATFORM            (20.0)

#define CLIMBER_CALIBRATION_COUNT               ((int)(0.1 * N1SEC))
#define CLIMB_DRIVE_FORWARD_ON_PLATFORM_TIME    ((int)(1.5 * N1SEC))
#define CLIMB_PREP_TIME                         ((int)(1.9 * N1SEC))
#define CLIMB_TWO_PREP_TIME                     ((int)(0.5 * N1SEC))

class Climber
{
    private:
        frc::Encoder *encoder;
		frc::VictorSP *climber;

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
        bool isEncoderBroken;
        int encoderCheckCount;
        int initialEncoderReading;
        bool encoderCheckFirstTime;
        bool isManualOverride;
        int atGoalCount;
        int climbStage;

        bool isCalibrating;
        bool encoderReset;
        int calibrationCount;
        int printCount;
        double togetherClimbGoalPosition;
        int climbCount;
        bool isClimbing;
        bool isLevTwoClimbing;


    public:
        Climber(TalonXX* pRobot);
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        double GetHeight(void);
        void UpdateDash(void);
        void Service(void);
        void ManualMove(double);
        void OverrideManualMove(double);
        void TurnOnOverride(void);
        void TurnOffOverride(void);
        bool isOverrideOn(void);
        void GiveGoalPosition(double);
        void ControlMove(void);
        double GetGoalPosition(void);
        bool AtFloor(void);
        bool AtMax(void);
        bool AtGoalPosition(void);
        void ClimbStop(void);
        void AutoClimb(void);
        void TurnOnAutoClimb(void);
        void TurnOffAutoClimb(void);
        void TurnOnLevTwoClimb(void);
        void TurnOffLevTwoClimb(void);
        void LevTwoAutoClimb(void);
};
#endif /*Climber_H_*/
