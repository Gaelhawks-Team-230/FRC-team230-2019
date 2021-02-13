#include "Common.h"
#include "PixyLine.h"

PixyLine::PixyLine(TalonXX* pRobot)
{
    lineLocator = new frc::AnalogInput(PIXY_LINE_ANALOG_INPUT);
    lineDetector = new frc::DigitalInput(PIXY_LINE_DETECTOR);

    LocalReset();
}

void PixyLine::LocalReset()
{
}

void PixyLine::StartingConfig()
{

}

void PixyLine::StopAll()
{

}

double PixyLine::LineLocation()
{
    double camVoltage;
    camVoltage = lineLocator->GetVoltage();
    return camVoltage;
}

bool PixyLine::LineDetected()
{
    if(lineDetector->Get())
    {
        return true;
    }
    else
    {
        return false;
    }
}


void PixyLine::UpdateDash()
{
    frc::SmartDashboard::PutBoolean("Line detected: ", LineDetected());
    frc::SmartDashboard::PutNumber("Line location: ", (double) LineLocation());
}

void PixyLine::Service()
{
    
}