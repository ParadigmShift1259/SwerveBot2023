
#include "PlaceHighCube.h"

#include "ConstantsDeploymentAngles.h"
#include <frc/smartdashboard/SmartDashboard.h>

PlaceHighCube::PlaceHighCube(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHighCube/startCommand");
  
  m_logStartCommand.Append(true);
}

void PlaceHighCube::Initialize()
{
  m_deployment.RetractBackPlate();
  m_deployment.RetractArm();
  m_deployment.RotateArmToAngle(kPlaceHighCubeAngle);
}

void PlaceHighCube::Execute()
{

}

bool PlaceHighCube::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kPlaceHighCubeAngle);
}

void PlaceHighCube::End(bool interrupted)
{
  m_deployment.ExtendArm();
  m_logStartCommand.Append(false);
}