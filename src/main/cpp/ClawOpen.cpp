#include "ClawOpen.h"

ClawOpen::ClawOpen(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});
  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/clawOpen/startCommand");
  m_logStartCommand.Append(true);
}

void ClawOpen::Execute()
{
  m_claw.Open();
}

bool ClawOpen::IsFinished()
{
  return true;
}

void ClawOpen::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}