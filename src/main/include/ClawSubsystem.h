#pragma once

#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "ConstantsDigitalOut.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class ClawSubsystem : public frc2::SubsystemBase
{
public:
    ClawSubsystem();
    void Periodic();
    void Open();
    void Close();
        
private:
    frc::Solenoid m_solenoid;
    frc::Timer m_timer;
};