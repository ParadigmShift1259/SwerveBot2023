#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

using namespace frc;

class RotateTurntableCW : public frc2::CommandHelper<frc2::CommandBase, RotateTurntableCW> {
 public:
  explicit RotateTurntableCW(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
 private:
  TurntableSubsystem& m_turntable;
  Timer m_timer;
};