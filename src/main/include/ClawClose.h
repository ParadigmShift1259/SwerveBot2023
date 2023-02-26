#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ClawClose : public frc2::CommandHelper<frc2::CommandBase, ClawClose>
{
public:
  explicit ClawClose(ISubsystemAccess& subsystemAccess);

  void Execute() override;
  bool IsFinished() override;
  
private:
  ClawSubsystem& m_claw;

  wpi::log::BooleanLogEntry m_logStartCommand;
};