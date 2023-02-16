#include "IntakeIngest.h"

IntakeIngest::IntakeIngest(ISubsystemAccess& subsystemAccess) 
  : m_intake(subsystemAccess.GetIntake())
  , m_turntable(subsystemAccess.GetTurntable())
{
  AddRequirements({&subsystemAccess.GetIntake(), &subsystemAccess.GetTurntable()});
}

void IntakeIngest::Execute()
{
  m_intake.Set(kIngestSpeed);
  m_turntable.SetTurnTable(kTurntableCCWSpeed);
}

void IntakeIngest::End(bool interrupted) 
{
  m_intake.Set(0.0);
  m_turntable.SetTurnTable(0.0);
}