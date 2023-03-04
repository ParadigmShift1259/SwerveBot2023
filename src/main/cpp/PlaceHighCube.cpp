
#include "PlaceHighCube.h"

#include "ConstantsDeploymentAngles.h"
#include <frc/smartdashboard/SmartDashboard.h>

PlaceHighCube::PlaceHighCube(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHighCube/startCommand");
  m_logAngle = wpi::log::DoubleLogEntry(log, "/placeHighCube/angle");
  
  m_logStartCommand.Append(true);
}

void PlaceHighCube::Execute()
{
  // degree_t angle = degree_t(SmartDashboard::GetNumber("GotoAngle", 0.0));

  m_deployment.RetractArm();
  m_deployment.RotateArmToAngle(kPlaceHighAngle);
  // m_logAngle.Append(angle.to<double>());
  // m_deployment.RotateArmToAngle(angle);
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