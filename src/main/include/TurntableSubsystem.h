#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/ControlMode.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "ConstantsDigitalOut.h"
#include "ConstantsCANIDs.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

using namespace frc;

constexpr double kTurntableCWSpeed = 0.75;
constexpr double kTurntableCCWSpeed = -0.75;
constexpr units::second_t kTurntableCWRunTime = 10.0_s;

class TurntableSubsystem : public frc2::SubsystemBase
{
public:
    TurntableSubsystem();

    /// Will be called periodically whenever the CommandScheduler runs.
    void Periodic() override;

    /// Turns the TurnTable at speed
    /// \param speed        Speed used in turning, between -1.0 and 1.0 with 0.0 as stopped
    void SetTurnTable(double speed);

private:
    /// 775 that shuffles balls around in the turntable
    TalonSRX m_turntablemotor;
};