#include "TravelPosition.h"

TravelPosition::TravelPosition(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
}

void TravelPosition::Execute()
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

bool TravelPosition::IsFinished()
{
    return m_deployment.IsAtDegreeSetpoint(kAcceptedPosition);
}

void TravelPosition::End(bool interrupted)
{
    m_deployment.Stop();
}