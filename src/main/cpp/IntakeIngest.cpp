#include "IntakeIngest.h"

#include <frc2/command/WaitCommand.h>

IntakeIngest::IntakeIngest(ISubsystemAccess& subsystemAccess) 
  : m_deployment(subsystemAccess.GetDeployment())
  , m_intake(subsystemAccess.GetIntake())
  , m_turntable(subsystemAccess.GetTurntable())
{
  AddRequirements({&subsystemAccess.GetDeployment(), &subsystemAccess.GetIntake(), &subsystemAccess.GetTurntable()});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/placeHigh/startCommand");
}

void IntakeIngest::Execute()
{
  m_intake.IntakeOut(true);
  m_deployment.ExtendBackPlate();
  // WaitCommand(0.5_s); Do we still need this? Rollers not touching
  m_intake.Set(kIngestSpeed);
  m_turntable.SetTurnTable(kTurntableCCWSpeed);
}

void IntakeIngest::End(bool interrupted) 
{
  m_intake.Set(0.0);
  m_turntable.SetTurnTable(0.0);
  m_logStartCommand.Append(false);
}