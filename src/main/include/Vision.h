#include <vector>
#include <wpi/DataLog.h>

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/DataLogManager.h>

#include <units/angle.h>
#include <units/length.h>
#include <iostream>

class Vision
{
    public:
    Vision(/* args */);
    void Periodic();

    std::vector<double> m_net_buffer{2};

    private:

    std::vector<double> m_zero_vector = {42.0, 42.0, 42.0, 92, 10, 22};

    std::shared_ptr<nt::NetworkTable> m_net_table = 
        nt::NetworkTableInstance::GetDefault().GetTable("limelight-twoplus");

  wpi::log::DoubleLogEntry m_logRobotPoseX;
  wpi::log::DoubleLogEntry m_logRobotPoseY;
  wpi::log::DoubleLogEntry m_logRobotPoseTheta;
};