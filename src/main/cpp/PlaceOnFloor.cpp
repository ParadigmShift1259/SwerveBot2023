#include "PlaceOnFloor.h"

#include "ConstantsDeploymentAngles.h"

PlaceOnFloor::PlaceOnFloor(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeOnFloor/startCommand");
}

void PlaceOnFloor::Execute()
{
    m_deployment.RetractArm();
    m_deployment.RotateArmToAngle(kPlaceOnFloorAngle);
}

bool PlaceOnFloor::IsFinished()
{
    return m_deployment.IsAtDegreeSetpoint(kPlaceOnFloorAngle);
}

void PlaceOnFloor::End(bool interrupted)
{
    m_deployment.ExtendArm();
    m_logStartCommand.Append(false);
}