#ifndef HATCHHANDLER_H_
#define HATCHHANDLER_H_

#include "Common.h"
#include <frc/DigitalInput.h>
#include <frc/VictorSP.h>

class TalonXX;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)
#define DELAY_CAM_RESET			(1)//((int)(0.1 * (N1SEC)))
#define DELAY_CAM_STOP          (1)//((int)(0.1 * (N1SEC)))
#define RELEASE_TIME            (COMMAND_ARRAY_SIZE)// (2 * LOOPSIZE) //((int)(1 * (N1SEC)))
#define LOOPSIZE                (2)
#define CAM_MAX_CMD                 (1.0)
#define COMMAND_ARRAY_SIZE      (3)
#define EJECT_TIME              ((int)(0.75 * (N1SEC))) //((int)(0.25 * (N1SEC)))
#define EJECT_BACKUP            ((int)(0.35 * (N1SEC))) //((int)(0.25 * (N1SEC)))
#define WAIT_TIME               ((int)(0.25 * (N1SEC)))
#define CLIMB_TURN_NUMBER       (15)
#define DRAG_DELAY              ((int)(0.25 * (N1SEC)))

class HatchHandler
{
    private:

        frc::VictorSP *cams;
        frc::VictorSP *vacuum;
        frc::DigitalInput *hatchSensor;
        TalonXX *mainRobot;
#ifdef USE_VACUUM
        double vacMotorCmd;
        bool vacOn;
        bool wasVacOn;
#endif
        double camMotorCmd;
        bool seesCam;
        int resetCount;
        bool isReleasing;
        bool isResetting;
        int loopCount;
        int stage;
        int loopStage;
        int climbTurnLoopCount;
        bool isFeederStation;
        int feederStationStage;
        int feederStationCount;
        int floorPickupStage;
        bool isFloorPickup;
        int camDownLoopCount;
        int camDownLoopStage;
        int camDownStage;
        bool isCamDown;
        bool currentReading;
        bool oldReading;
        bool firstTimeThrough;
#ifdef USE_VACUUM
        int vacFloorPickupCount;
#endif

        float cmdArray[COMMAND_ARRAY_SIZE] = {-0.5, 0.0, 0.0};

    public:
        HatchHandler(TalonXX* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void MoveToFloor(void);
        void PickupHatch(void);
        void ReleaseHatch(void);
        void StopReleaseHatch(void);
        void ResetCams(void);
        bool SeeCam(void);
        void UpdateDash(void);
        void Service(void);
        void FeederStation(void);
        void TurnOnFeederStationRelease(void);
        void TurnOffFeederStationRelease(void);
        void FloorPickUp(void);
        void TurnOnFloorPickup(void);
        void AutomaticHatchRelease(void);
#ifdef USE_VACUUM
        void StartVacuum(void);
        void HoldVacuum(void);
        void ReleaseVacuum(void);
#endif
};

#endif /*HATCHHANDLER_H_*/
