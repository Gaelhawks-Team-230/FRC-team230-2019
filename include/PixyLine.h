#ifndef PIXYLINE_H_
#define PIXYLINE_H_

#include "Common.h"
#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>

class TalonXX;

#define TELE_LINE_ROTATE_MULTIPLER       (-0.5)

class PixyLine
{
    private:

        TalonXX *mainRobot;
        frc::AnalogInput *lineLocator;
        frc::DigitalInput *lineDetector;

  

    public:
        PixyLine(TalonXX* pRobot);
        void LocalReset(void);
        void StartingConfig(void);
        void StopAll(void);
        bool LineDetected(void);
        double LineLocation(void);
        void UpdateDash(void);
        void Service(void);
};
#endif 
