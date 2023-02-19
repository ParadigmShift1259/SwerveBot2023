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
    //m_deployment.RotateArmToAngle(kPlaceLowAngle);

    if (m_deployment.CurrentDegreePosition() < kPlaceLowAngle)
    {
        m_deployment.RotateOutOfFrame(kRotateSpeed);
    }
    else
    {
        m_deployment.RotateIntoFrame(kRotateSpeed);
    }
}

bool PlaceLow::IsFinished()
{
    return m_deployment.IsAtDegreeSetpoint(kPlaceLowAngle);
}

void PlaceLow::End(bool interrupted)
{
    //m_deployment.Stop();
}