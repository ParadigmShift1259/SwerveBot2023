#include "ClawClose.h"

ClawClose::ClawClose(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/clawClose/startCommand");
}

void ClawClose::Initialize()
{
  m_logStartCommand.Append(true);
}

void ClawClose::Execute()
{
  m_claw.Close();
}

bool ClawClose::IsFinished()
{
  return true;
}

void ClawClose::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}