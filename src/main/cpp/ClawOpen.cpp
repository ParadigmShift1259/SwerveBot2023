#include "ClawOpen.h"

ClawOpen::ClawOpen(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  printf("ClawOpen::ClawOpen\n");
  AddRequirements({&subsystemAccess.GetClaw()});
  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/clawOpen/startCommand");
}

void ClawOpen::Execute()
{
  printf("ClawOpen::Execute %u requirements\n", GetRequirements().size());

  m_claw.Open();
  m_logStartCommand.Append(false);
}

bool ClawOpen::IsFinished()
{
  return true;
}