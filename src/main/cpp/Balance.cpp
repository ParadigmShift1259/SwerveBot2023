#include "Balance.h"

Balance::Balance(DriveSubsystem& driveSubsystem, ISubsystemAccess& subsystemAccess) 
  : m_drive(driveSubsystem)
  , m_timer()
{
  AddRequirements({&driveSubsystem});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/balance/startCommand");
}

void Balance::Initialize()
{
  m_timer.Reset();
  m_timer.Start();
}

void Balance::Execute()
{
  m_logStartCommand.Append(false);
  double driveSpeed = std::clamp(kMaxAutoBalanceSpeed * m_drive.GetPitch() / (kMaxPitch * m_timer.Get().to<double>()), -kMaxAutoBalanceSpeed, kMaxAutoBalanceSpeed);
  m_drive.Drive(units::velocity::meters_per_second_t(driveSpeed), 0.0_mps, 0.0_rad_per_s, false); 
}

bool Balance::IsFinished()
{
  return abs(m_drive.GetPitch()) < 5.0;
}

void Balance::End(bool interrupted)
{
  m_drive.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
}
