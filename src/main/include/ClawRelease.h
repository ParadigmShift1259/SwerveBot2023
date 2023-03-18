#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ClawRelease : public frc2::CommandHelper<frc2::CommandBase, ClawRelease>
{
public:
  explicit ClawRelease(ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  ClawSubsystem& m_claw;

  wpi::log::BooleanLogEntry m_logStartCommand;
};