#ifndef SAMPLE_H_
#define SAMPLE_H_


#include "Common.h"

class TalonXX;

//#defines
//example: #define LEVEL_ONE_HEIGHT     (23.0)

class Sample
{
    private:
       // Create objects needed by this class
		// example: VictorSP *sampleMotor;

        TalonXX *mainRobot;

        //declare member variables
        //example: float height;

    public:
        Sample(TalonXX* pRobot);
        //Functions
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void DoAThing(int);
        void UpdateDash(void);
        void Service(void);
        void ControlSystem(void);
};
#endif /*SJElevator_H_*/
