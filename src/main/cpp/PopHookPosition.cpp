#include "PopHookPosition.h"

#include <frc2/command/WaitCommand.h>

#include "ConstantsDeploymentAngles.h"

PopHookPosition::PopHookPosition(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
  , m_deployment(subsystemAccess.GetDeployment())
  , m_timer()
{
  AddRequirements({&subsystemAccess.GetClaw(), &subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/PopHookPosition/startCommand");
}

void PopHookPosition::Initialize()
{
  m_logStartCommand.Append(true);
}

void PopHookPosition::Execute()
{
  m_deployment.RotateArmToAngle(kHookPopAngle);
}

bool PopHookPosition::IsFinished()
{
  return m_deployment.IsAtDegreeSetpoint(kHookPopAngle);
}

void PopHookPosition::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}