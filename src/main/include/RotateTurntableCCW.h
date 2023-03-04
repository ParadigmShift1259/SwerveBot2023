#pragma once

#include <frc/Timer.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

using namespace frc;

class RotateTurntableCCW : public frc2::CommandHelper<frc2::CommandBase, RotateTurntableCCW>
{
public:
  explicit RotateTurntableCCW(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  TurntableSubsystem& m_turntable;
  Timer m_timer;

  wpi::log::BooleanLogEntry m_logStartCommand;
};