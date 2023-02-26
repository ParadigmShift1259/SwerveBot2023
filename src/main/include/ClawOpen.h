#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "ISubsystemAccess.h"

class ClawOpen : public frc2::CommandHelper<frc2::CommandBase, ClawOpen>
{
public:
  explicit ClawOpen(ISubsystemAccess& subsystemAccess);

  void Execute() override;
  bool IsFinished() override;
  
private:
  ClawSubsystem& m_claw;

  wpi::log::BooleanLogEntry m_logStartCommand;
};