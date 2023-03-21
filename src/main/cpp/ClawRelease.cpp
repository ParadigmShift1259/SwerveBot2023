#include "ClawRelease.h"

ClawRelease::ClawRelease(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});
  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ClawRelease/startCommand");
}

void ClawRelease::Initialize()
{
  m_logStartCommand.Append(true);
}

void ClawRelease::Execute()
{
  m_claw.Unclamp();
  m_claw.Release();
}

bool ClawRelease::IsFinished()
{
  return true;
}

void ClawRelease::End(bool interrupted)
{
  m_logStartCommand.Append(false);
}