#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "DriveSubsystem.h"

class Balance : public frc2::CommandHelper<frc2::CommandBase, Balance>
{
public:
  explicit Balance(DriveSubsystem& driveSubsystem);

  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  DriveSubsystem& m_drive;

  wpi::log::BooleanLogEntry m_logStartCommand;

  static constexpr double kMaxAutoBalanceSpeed = 1.0;
  static constexpr double kMaxPitch = 15.0;
};