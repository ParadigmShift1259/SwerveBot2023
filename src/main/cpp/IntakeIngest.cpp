#include "IntakeIngest.h"

#include <frc2/command/WaitCommand.h>

IntakeIngest::IntakeIngest(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
  , m_intake(subsystemAccess.GetIntake())
  , m_turntable(subsystemAccess.GetTurntable())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake(), &subsystemAccess.GetTurntable()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/intakeIngest/startCommand");
  m_logStartCommand.Append(true);
}

void IntakeIngest::Execute()
{
  m_intake.ExtendIntake();
  m_deployment.ExtendBackPlate();
  frc2::WaitCommand(0.25_s); // Wait for backplate to extend and turntable motor to engage
  m_intake.Set(kIngestSpeed);
  m_turntable.SetTurnTable(kTurntableCCWSpeed);
}

void IntakeIngest::End(bool interrupted) 
{
  m_intake.Set(0.0);
  m_turntable.SetTurnTable(0.0);
  m_logStartCommand.Append(false);
}