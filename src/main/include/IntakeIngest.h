#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeIngest : public frc2::CommandHelper<frc2::CommandBase, IntakeIngest> {
 public:
  explicit IntakeIngest(ISubsystemAccess& subsystemAccess);

  void Execute() override;
  void End(bool interrupted) override;
  
 private:
  IntakeSubsystem& m_intake;
  TurntableSubsystem& m_turntable;
};