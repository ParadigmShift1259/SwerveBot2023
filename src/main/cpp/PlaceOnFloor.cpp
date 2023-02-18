#include "PlaceOnFloor.h"

#include "ConstantsDeploymentAngles.h"

PlaceOnFloor::PlaceOnFloor(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});
}

void PlaceOnFloor::Execute()
{
    m_deployment.ExtendArm();
    if (m_deployment.CurrentDegreePosition() < kPlaceOnFloorAngle)
    {
        m_deployment.RotateOutOfFrame(0.2);
    }
    else
    {
        m_deployment.RotateIntoFrame(0.2);
    }
}

bool PlaceOnFloor::IsFinished()
{
    return m_deployment.IsAtDegreeSetpoint(kPlaceOnFloorAngle);
}

void PlaceOnFloor::End(bool interrupted)
{
    m_deployment.Stop();
}