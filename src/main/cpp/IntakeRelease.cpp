
#include "IntakeRelease.h"

IntakeRelease::IntakeRelease(ISubsystemAccess& subsystemAccess)
 : m_deployment(subsystemAccess.GetDeployment())
 , m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeRelease/startCommand");
}

void IntakeRelease::Execute() {
  m_intake.Set(kReleaseSpeed);
}

bool IntakeRelease::IsFinished()
{
  return true;
}

void IntakeRelease::End(bool interrupted) {
  m_intake.Set(0);
  // Only retract intake if deployment arm is out of the way
  // m_intake.IntakeOut(!m_deployment.IsOkayToRetractIntake());
  //TODO replace with m_intake.RetractIntake();
  m_logStartCommand.Append(false);
}
