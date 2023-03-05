
#include "ReleaseHigh.h"

#include "ConstantsDeploymentAngles.h"
#include <frc/smartdashboard/SmartDashboard.h>

ReleaseHigh::ReleaseHigh(ISubsystemAccess& subsystemAccess)\
  : m_claw(subsystemAccess.GetClaw())
  , m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetClaw(), &subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ReleaseHigh/startCommand");
  m_logAngle = wpi::log::DoubleLogEntry(log, "/ReleaseHigh/angle");
  
  m_logStartCommand.Append(true);
}

void ReleaseHigh::Initialize()
{
  m_deployment.RotateArmToAngle(kReleaseHighAngle);
}

void ReleaseHigh::Execute()
{

}

bool ReleaseHigh::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kReleaseHighAngle);
}

void ReleaseHigh::End(bool interrupted)
{
  m_claw.Open();
  m_logStartCommand.Append(false);
}