#include "ClawSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

ClawSubsystem::ClawSubsystem()
    : m_motor(kClawCANID, CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_photoeye(kClawPhotoeye)
    , m_solenoid(PneumaticsModuleType::CTREPCM, kClawSolenoid)
{
    m_motor.RestoreFactoryDefaults();
    m_motor.EnableVoltageCompensation(12.0);
    m_motor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_motor.SetSmartCurrentLimit(25);
}

void ClawSubsystem::Periodic()
{
    SmartDashboard::PutNumber("Claw Output Curret", m_motor.GetOutputCurrent());
    SmartDashboard::PutNumber("Claw RPM", m_enc.GetVelocity());
    SmartDashboard::PutBoolean("Claw PhotoEye Active", IsPhotoeyeActive());
}

void ClawSubsystem::Ingest()
{
    m_motor.Set(kClawIngestSpeed);
}

void ClawSubsystem::IngestCube()
{
    m_motor.Set(kClawIngestCubeSpeed);
}

void ClawSubsystem::Release()
{
    m_motor.Set(kClawReleaseSpeed);
}

void ClawSubsystem::Hold()
{
    m_motor.Set(kClawHoldSpeed);
}

void ClawSubsystem::Stop()
{
    m_motor.Set(0.0);
}

bool ClawSubsystem::IsPhotoeyeActive()
{
    return m_photoeye.Get();
}

void ClawSubsystem::Unclamp()
{
    m_motor.Set(kClawHoldSpeed);
    m_solenoid.Set(true);
}

void ClawSubsystem::Clamp()
{
    m_solenoid.Set(false);
    m_motor.Set(0.0);
}