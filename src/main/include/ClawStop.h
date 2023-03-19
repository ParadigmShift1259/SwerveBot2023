#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ClawStop : public frc2::CommandHelper<frc2::CommandBase, ClawStop>
{
public:
  explicit ClawStop(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  ClawSubsystem& m_claw;

  wpi::log::BooleanLogEntry m_logStartCommand;
};