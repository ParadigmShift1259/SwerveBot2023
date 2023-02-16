#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class IntakeStop : public frc2::CommandHelper<frc2::CommandBase, IntakeStop> {
 public:
  explicit IntakeStop(ISubsystemAccess& subsystemAccess, bool retractIntake);

  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;

 private:
  IntakeSubsystem& m_intake;
  bool m_retractIntake;
};