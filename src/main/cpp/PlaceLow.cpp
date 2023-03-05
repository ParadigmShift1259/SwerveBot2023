#include "PlaceLow.h"

#include "ConstantsDeploymentAngles.h"

PlaceLow::PlaceLow(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeLow/startCommand");
  m_logStartCommand.Append(true);
}

void PlaceLow::Initialize()
{
  m_deployment.RetractBackPlate(); 
  m_deployment.RetractArm();
  m_deployment.RotateArmToAngle(kPlaceLowAngle);
}

void PlaceLow::Execute()
{

}

bool PlaceLow::IsFinished()
{
  return true;// m_deployment.IsAtDegreeSetpoint(kPlaceLowAngle);
}

void PlaceLow::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}