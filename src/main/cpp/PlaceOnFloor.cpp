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
//    m_deployment.RotateArmToAngle(kPlaceOnFloorAngle);
    m_deployment.RotateArmToAngle(120_deg);

    // if (m_deployment.CurrentDegreePosition() < kPlaceOnFloorAngle)
    // {
    //     m_deployment.RotateOutOfFrame(kRotateSpeed);
    // }
    // else
    // {
    //     m_deployment.RotateIntoFrame(kRotateSpeed);
    // }
}

bool PlaceOnFloor::IsFinished()
{
    return true;//m_deployment.IsAtDegreeSetpoint(kPlaceOnFloorAngle);
}

void PlaceOnFloor::End(bool interrupted)
{
    //m_deployment.Stop();
}