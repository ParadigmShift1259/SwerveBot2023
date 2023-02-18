#include "PlaceHigh.h"

PlaceHigh::PlaceHigh(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
}

void PlaceHigh::Execute()
{
    m_deployment.Extend();
    m_deployment.RotateOutOfFrame(0.2);
}

bool PlaceHigh::IsFinished()
{
    return m_deployment.IsForwardLimitSwitchClosed();
}

void PlaceHigh::End(bool interrupted)
{
    m_deployment.Stop();
}