#include "IntakeIngest.h"

#include <frc2/command/WaitCommand.h>

IntakeIngest::IntakeIngest(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
  , m_intake(subsystemAccess.GetIntake())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeIngest/startCommand");
}

void IntakeIngest::Initialize()
{
  m_logStartCommand.Append(true);
}

void IntakeIngest::Execute()
{
  if (m_deployment.IsInFrame())
  {
    m_intake.ExtendIntake();
    m_intake.Set(kIngestSpeed);
  }
}

bool IntakeIngest::IsFinished()
{
  return m_intake.IsPhotoeyeActive();
}

void IntakeIngest::End(bool interrupted) 
{
  m_intake.Set(0.0);
  m_logStartCommand.Append(false);
}