
#pragma once

#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "ConstantsDigitalOut.h"
#include "ConstantsCANIDs.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

constexpr double kIngestSpeed = 1.0;
constexpr double kReleaseSpeed = -0.80;

class IntakeSubsystem : public frc2::SubsystemBase
{
public:

    IntakeSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Drives the intake at a given speed
    /// \param speed         Desired motor speed to run, ranging from [-1, 1]
    void Set(double speed);

    void IntakeOut(bool out);
private:
    /// 775 that runs intake
    TalonSRX m_motor;
    frc::Solenoid m_solenoid;
    frc::Timer m_timer;
};