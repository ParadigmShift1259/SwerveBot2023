#include "ReleaseCone.h"

ReleaseCone::ReleaseCone(ISubsystemAccess& subsystemAccess)
  : m_claw(subsystemAccess.GetClaw())
  , m_deployment(subsystemAccess.GetDeployment())
{
  AddRequirements({&subsystemAccess.GetClaw(), &subsystemAccess.GetDeployment()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/ReleaseCone/startCommand");
  m_logPosition = wpi::log::DoubleLogEntry(log, "/ReleaseCone/position");
}

void ReleaseCone::Initialize()
{
  m_logStartCommand.Append(true);

  if (m_deployment.IsAtSetpoint(kPlaceHighAbsolute))
  {
    m_releasePosition = kReleaseHighPosition;
    m_releaseAbsolute = kReleaseHighAbsolute;
  }
  else if (m_deployment.IsAtSetpoint(kPlaceLowAbsolute))
  {
    m_releasePosition = kReleaseLowPosition;
    m_releaseAbsolute = kReleaseLowAbsolute;
  }
  else
  {
    printf("NOT AT PLACE ANGlE");
    return;
  }
  m_deployment.RotateArm(m_releasePosition);
  m_logPosition.Append(m_releasePosition);
}

void ReleaseCone::Execute()
{
  m_deployment.ResetEncoderWithAbsolute();
}

bool ReleaseCone::IsFinished()
{
  return m_deployment.IsAtSetpoint(m_releaseAbsolute);
}

void ReleaseCone::End(bool interrupted)
{
  if (IsFinished() && !interrupted)
  {
    m_claw.Release();
  }
  m_logStartCommand.Append(false);
}