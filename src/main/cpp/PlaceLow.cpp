#include "PlaceLow.h"

PlaceLow::PlaceLow(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
}

void PlaceLow::Execute()
{
    m_deployment.Retract();
    if (m_deployment.CurrentDegreePosition() < kAcceptedPosition)
    {
        m_deployment.RotateOutOfFrame(0.2);
    }
    else
    {
        m_deployment.RotateIntoFrame(0.2);
    }
}

bool PlaceLow::IsFinished()
{
    return m_deployment.IsAtDegreeSetpoint(kAcceptedPosition);
}

void PlaceLow::End(bool interrupted)
{
    m_deployment.Stop();
}