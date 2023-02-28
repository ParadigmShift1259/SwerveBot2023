#include "Balance.h"

Balance::Balance(DriveSubsystem& driveSubsystem) 
  : m_drive(driveSubsystem)
{
  AddRequirements({&driveSubsystem});

  // wpi::log::DataLog& log = subsystemAccess.GetLogger();
  // m_logStartCommand = wpi::log::BooleanLogEntry(log, "/balance/startCommand");
}

void Balance::Execute()
{
  // m_logStartCommand.Append(false);
  double driveSpeed = std::clamp( kMaxAutoBalanceSpeed *  m_drive.GetPitch() / kMaxPitch, -kMaxAutoBalanceSpeed, kMaxAutoBalanceSpeed);
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
