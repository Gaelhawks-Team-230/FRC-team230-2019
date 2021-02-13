#include "Common.h"
#include "Sample.h"

Sample::Sample(TalonXX* pRobot)
{
    mainRobot = pRobot;
    LocalReset();
}

void Sample::LocalReset()
{
}

void Sample::StartingConfig()
{

}

void Sample::StopAll()
{

}

void Sample::DoAThing(int a)
{

}

//Anything that you want printed on the dashboard for testing or during match. 
//Any information which may be important. 
void Sample::UpdateDash()
{

}
//Called every loop
void Sample::Service()
{
    
}

void Sample::ControlSystem()
{
    
}