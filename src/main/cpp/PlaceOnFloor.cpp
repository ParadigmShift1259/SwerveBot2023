#include "PlaceOnFloor.h"

#include "ConstantsDeploymentAngles.h"

PlaceOnFloor::PlaceOnFloor(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeOnFloor/startCommand");
  m_logStartCommand.Append(true);
}

void PlaceOnFloor::Initialize()
{
  m_deployment.RetractBackPlate();
  m_deployment.RetractArm();
  m_deployment.RotateArmToAngle(kPlaceOnFloorAngle);
}

void PlaceOnFloor::Execute()
{

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