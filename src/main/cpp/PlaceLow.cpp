#include "PlaceLow.h"

#include "ConstantsDeploymentAngles.h"

PlaceLow::PlaceLow(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
}

void PlaceLow::Execute()
{
    m_deployment.RetractArm();
    if (m_deployment.CurrentDegreePosition() < kPlaceLowAngle)
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
    return m_deployment.IsAtDegreeSetpoint(kPlaceLowAngle);
}

void PlaceLow::End(bool interrupted)
{
    m_deployment.Stop();
}