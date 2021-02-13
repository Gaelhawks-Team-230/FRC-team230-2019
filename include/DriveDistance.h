/*
DriveDistance.cpp

Created by Suhaas (Feb 2, 2019)


*/

#ifndef SRC_DRIVEDISTANCE_H_
#define SRC_DRIVEDISTANCE_H_


#include "Common.h"
#include "TalonXX_main.h"
#include <frc/WPILib.h>

class TalonXX;

#define WHEEL_CIRCUMFERENCE    (19.687 * 0.95)  //inches
#define ENCODER_PULSE_COUNT    (360.0)

class DriveDistance
{
    private:
       // Create objects needed by this class
		// example: VictorSP *DriveDistanceMotor;

        TalonXX *mainRobot;

        frc::Encoder *encoderFL;
        frc::Encoder *encoderFR;
        frc::Encoder *encoderBL;
        frc::Encoder *encoderBR;

        double fwdDis;
        double leftDis;
        double rightDis;
        double diagDis;
        double LsideDis;
        double RsideDis;
        double averageDis;
        double frontDis;
        double backDis;

        double distancePerPulse;
        const double ENCODER_PULSE_CHECK = 80;

    public:
        DriveDistance(TalonXX* pRobot);
        //Functions
        void LocalReset(void);
        void ResetEncoders(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);
        double GetLeftDistance(void);
        double GetRightDistance(void);
        double GetStrafingDistance(void);
        double GetForwardsDistance(void);
        bool CheckEncoders(int encoderCase1, int encoderCase2);
        bool EncoderStatus(void);
        void StartingConfig(void);
};
#endif /*Drivedistance_H_*/
