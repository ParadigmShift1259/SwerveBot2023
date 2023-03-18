#include "ClawSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace frc;

ClawSubsystem::ClawSubsystem()
    : m_motor(kClawCANID, CANSparkMaxLowLevel::MotorType::kBrushless)
    , m_photoeye(kClawPhotoeye)
{
    m_motor.RestoreFactoryDefaults();
    m_motor.SetIdleMode(CANSparkMax::IdleMode::kBrake);
    m_motor.SetSmartCurrentLimit(25);
}

void ClawSubsystem::Periodic()
{
    SmartDashboard::PutNumber("Claw Output Curret", m_motor.GetOutputCurrent());
}

void ClawSubsystem::Ingest()
{
    m_motor.Set(kClawIngestSpeed);
}

void ClawSubsystem::Release()
{
    m_motor.Set(kClawReleaseSpeed);
}

void ClawSubsystem::Hold()
{
    m_motor.Set(0.2);
}

void ClawSubsystem::Stop()
{
    m_motor.Set(0.0);
}

bool ClawSubsystem::IsPhotoeyeActive()
{
    return m_photoeye.Get();
}