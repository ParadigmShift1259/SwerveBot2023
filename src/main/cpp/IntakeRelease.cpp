
#include "IntakeRelease.h"

IntakeRelease::IntakeRelease(ISubsystemAccess& subsystemAccess)
 : m_deployment(subsystemAccess.GetDeployment())
 , m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeRelease/startCommand");
  m_logStartCommand.Append(true);
}

void IntakeRelease::Execute()
{
  m_intake.ExtendIntake();
  m_intake.Set(kReleaseSpeed);
}

bool IntakeRelease::IsFinished()
{
  return true;
}

void IntakeRelease::End(bool interrupted) {
  m_intake.Set(0);
  m_logStartCommand.Append(false);
}
