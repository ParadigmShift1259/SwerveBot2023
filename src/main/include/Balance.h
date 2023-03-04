#pragma once

#include <frc/Timer.h>

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "DriveSubsystem.h"
#include "ISubsystemAccess.h"

class Balance : public frc2::CommandHelper<frc2::CommandBase, Balance>
{
public:
  explicit Balance(DriveSubsystem& driveSubsystem, ISubsystemAccess& subsystemAccess);

  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  
private:
  DriveSubsystem& m_drive;
  frc::Timer m_speedTimer;
  frc::Timer m_endTimer;
  wpi::log::BooleanLogEntry m_logStartCommand;

  static constexpr double kMaxAutoBalanceSpeed = 1.5;
  static constexpr double kMaxPitch = 15.0;
};