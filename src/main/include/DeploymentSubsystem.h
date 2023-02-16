
#pragma once

#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "ConstantsDigitalOut.h"
#include "ConstantsCANIDs.h"

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

class DeploymentSubsystem : public frc2::SubsystemBase 
{

    public:
        DeploymentSubsystem();

        /// Will be called periodically whenever the CommandScheduler runs.
        void Periodic() override;

        /// Drives the deployment mechanism at a given speed
        /// \param speed         Desired motor speed to run, ranging from [-1, 1]
        void Set(double speed);

    private:
        TalonSRX m_motor;
        frc::Solenoid m_solenoid;
        frc::Timer m_timer;

};