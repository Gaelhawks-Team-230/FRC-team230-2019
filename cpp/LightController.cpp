#include "Common.h"
#include "LightController.h"

LightController::LightController(TalonXX* pRobot)
{
    light1 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_3);
	light2 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_2);
	light3 = new frc::DigitalOutput(LED_LIGHT_OUTPUT_1);
    
    mainRobot = pRobot;
    LocalReset();
}

void LightController::LocalReset()
{
    light1->Set(true);
    light2->Set(true);        
    light3->Set(true);
    lastTarget = false;
    currentTarget = false;
}

void LightController::StartingConfig()
{
    LocalReset();
}

void LightController::StopAll()
{
    LocalReset();
}


//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void LightController::UpdateDash()
{

}
//Called every loop
void LightController::Service()
{
    lastTarget = currentTarget;
    currentTarget = mainRobot->isTracking && mainRobot->isLineDetected;
    if(mainRobot->climbPrepped)
    {
        //Spaceship
        light1->Set(false);
        light2->Set(false);
        light3->Set(false);
    }
    else if(mainRobot->floorPickupPrepped)
    {
        light1->Set(true);
        light2->Set(false);
        light3->Set(false);
    }
    else if(mainRobot->theClaw->ReturnIsGrabbingCargoTilt())
    {
        light1->Set(true);
        light2->Set(false);
        light3->Set(true);
    }
    else if(lastTarget || currentTarget)
    {
        light1->Set(false);
        light2->Set(true);
        light3->Set(false);
    }
    else if(mainRobot->isAutoManualOverride)
    {
        if(mainRobot->isBlueAlliance)
        {
            light1->Set(false);
            light2->Set(true);
            light3->Set(true);
        }
        else
        {
            light1->Set(false);
            light2->Set(false);
            light3->Set(true);
        }
    }
    else if(mainRobot->isAuto)
    {
        light1->Set(true);
        light2->Set(true);
        light3->Set(true);
    }
    else
    {
        if(mainRobot->isBlueAlliance)
        {
            light1->Set(false);
            light2->Set(true);
            light3->Set(true);
        }
        else
        {
            light1->Set(false);
            light2->Set(false);
            light3->Set(true);
        }
    }
    
}