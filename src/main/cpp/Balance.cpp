#include "Balance.h"

#include <cmath> // For sqrt function

Balance::Balance(DriveSubsystem& driveSubsystem, ISubsystemAccess& subsystemAccess) 
  : m_drive(driveSubsystem)
  , m_speedTimer()
  , m_endTimer()
{
  AddRequirements({&driveSubsystem});

  wpi::log::DataLog& log = subsystemAccess.GetLogger();
  m_logStartCommand = wpi::log::BooleanLogEntry(log, "/balance/startCommand");
  m_logStartCommand.Append(true);
}

void Balance::Initialize()
{
  m_speedTimer.Reset();
  m_speedTimer.Start();
}

void Balance::Execute()
{
  double driveSpeed = std::clamp(kMaxAutoBalanceSpeed * m_drive.GetPitch() / (kMaxPitch * m_speedTimer.Get().to<double>()), -kMaxAutoBalanceSpeed, kMaxAutoBalanceSpeed);
  printf("Balance::Execute, Pitch %.3f, Speed %.3f, SpeedTime %.3f, EndTime %.3f\n", m_drive.GetPitch(), driveSpeed, m_speedTimer.Get().to<double>(), m_endTimer.Get().to<double>());
  m_drive.Drive(units::velocity::meters_per_second_t(driveSpeed), 0.0_mps, 0.0_rad_per_s, false);

  if (abs(m_drive.GetPitch()) < 5.0)
  {
    m_endTimer.Start();
    m_speedTimer.Reset();
  }
  else
  {
    m_endTimer.Reset();
    m_endTimer.Stop();
  }
}

bool Balance::IsFinished()
{
  return m_endTimer.Get() > 5.0_s;
}

void Balance::End(bool interrupted)
{
  m_logStartCommand.Append(false);
  m_drive.Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, false);
}
