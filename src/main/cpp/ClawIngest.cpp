#include "ClawIngest.h"

ClawIngest::ClawIngest(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ClawIngest/startCommand");
}

void ClawIngest::Initialize()
{
  m_logStartCommand.Append(true);
}

void ClawIngest::Execute()
{
  m_claw.Unclamp();
  m_claw.Ingest();
}

bool ClawIngest::IsFinished()
{
  return m_claw.IsPhotoeyeActive();
}

void ClawIngest::End(bool interrupted)
{
  // if (interrupted) // Button released
  // {
  //   m_claw.Stop();
  // }
  // else
  // {
    // m_claw.Hold();
  // }

  if (!interrupted)
  {
    m_claw.Clamp();
  }

  m_logStartCommand.Append(false);
}