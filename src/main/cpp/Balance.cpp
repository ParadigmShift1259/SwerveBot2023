#include "Balance.h"

#include <cmath> // For sqrt function
#include <frc/smartdashboard/SmartDashboard.h>

Balance::Balance(DriveSubsystem& driveSubsystem, ISubsystemAccess& subsystemAccess) 
  : m_drive(driveSubsystem)
  , m_speedTimer()
  , m_endTimer()
{
  AddRequirements({&driveSubsystem});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/balance/startCommand");
}

void Balance::Initialize()
{
  m_logStartCommand.Append(true);

  m_speedTimer.Reset();
  m_speedTimer.Start();
}

void Balance::Execute()
{
  // double balanceTolerance = frc::SmartDashboard::GetNumber("Balance Tolerance", kBalanceTolerance);
  // double maxAutoBalanceSpeed = frc::SmartDashboard::GetNumber("MaxAutoBalanceSpeed", kMaxAutoBalanceSpeed);
  double driveSpeed = std::clamp(kMaxAutoBalanceSpeed * m_drive.GetPitch() / (kMaxPitch), -kMaxAutoBalanceSpeed, kMaxAutoBalanceSpeed);

  if (abs(m_drive.GetPitch()) < kBalanceTolerance)
  {
    driveSpeed = std::clamp(kMaxAutoBalanceSpeed * m_drive.GetPitch() / (kMaxPitch * m_speedTimer.Get().to<double>()), -kMaxAutoBalanceSpeed, kMaxAutoBalanceSpeed);
    m_endTimer.Start();
  }
  else
  {
    m_endTimer.Reset();
    m_endTimer.Stop();
  }

  m_drive.Drive(units::velocity::meters_per_second_t(driveSpeed), 0.0_mps, 0.0_rad_per_s, false);
}

bool Balance::IsFinished()
{
  // double endTime = frc::SmartDashboard::GetNumber("BalanceEndTime", kBalanceEndTime.to<double>());
  // return m_endTimer.Get() > second_t(endTime);
  return m_endTimer.Get() > kBalanceEndTime;
}

void Balance::End(bool interrupted)
{
  m_logStartCommand.Append(false);
  m_drive.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
}
