#include "ClawIngestCube.h"

ClawIngestCube::ClawIngestCube(ISubsystemAccess& subsystemAccess) 
  : m_claw(subsystemAccess.GetClaw())
{
  AddRequirements({&subsystemAccess.GetClaw()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ClawIngestCube/startCommand");
}

void ClawIngestCube::Initialize()
{
  m_logStartCommand.Append(true);
}

void ClawIngestCube::Execute()
{
  m_claw.IngestCube();
}

bool ClawIngestCube::IsFinished()
{
  return false;
}

void ClawIngestCube::End(bool interrupted)
{
  m_claw.Hold();

  m_logStartCommand.Append(false);
}