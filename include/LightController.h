#ifndef LIGHTCONTROLLER_H_
#define LIGHTCONTROLLER_H_

#include "Common.h"

class TalonXX;


class LightController
{
    private:
  
        frc::DigitalOutput *light1;
        frc::DigitalOutput *light2;
        frc::DigitalOutput *light3;

        TalonXX *mainRobot;
        bool lastTarget;
        bool currentTarget;
   
    public:
        LightController(TalonXX* pRobot);
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        void UpdateDash(void);
        void Service(void);

};
#endif /*LIGHTCONTROLLER_H_*/
