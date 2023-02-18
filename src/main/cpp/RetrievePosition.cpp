#include "RetrievePosition.h"

RetrievePosition::RetrievePosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
}

void RetrievePosition::Execute()
{
    m_deployment.Extend();
    m_deployment.RotateIntoFrame(0.2);
}

bool RetrievePosition::IsFinished()
{
    return m_deployment.IsReverseLimitSwitchClosed();
}

void RetrievePosition::End(bool interrupted)
{
    m_deployment.Stop();
}