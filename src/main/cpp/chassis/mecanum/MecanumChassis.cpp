
//====================================================================================================================================================
// Copyright 2022 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#include <cmath>
#include <string>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>

#include <units/angle.h>

#include <chassis/holonomic/FieldDriveUtils.h>
#include <hw/factories/PigeonFactory.h>
#include <hw/DragonPigeon.h>
#include <utils/ConversionUtils.h>
#include <utils/Logger.h>

#include <chassis/mecanum/MecanumChassis.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>

using namespace std;
using namespace frc;
using namespace ctre::phoenix::motorcontrol::can;

MecanumChassis::MecanumChassis
(
    shared_ptr<IDragonMotorController>             leftFrontMotor, 
    shared_ptr<IDragonMotorController>             leftBackMotor, 
    shared_ptr<IDragonMotorController>             rightFrontMotor,
    shared_ptr<IDragonMotorController>             rightBackMotor,
    units::meter_t                                 wheelBase,
    units::meter_t                                 trackWidth,
    units::velocity::meters_per_second_t           maxSpeed,
    units::angular_velocity::degrees_per_second_t  maxAngSpeed,
    units::length::inch_t                          wheelDiameter,
    string                                         networktablename
) : m_leftFrontMotor(leftFrontMotor),
    m_leftBackMotor(leftBackMotor),
    m_rightFrontMotor(rightFrontMotor),
    m_rightBackMotor(rightBackMotor),
    m_pigeon(PigeonFactory::GetFactory()->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT)),
    m_maxSpeed(maxSpeed),
    m_maxAngSpeed(maxAngSpeed), 
    m_wheelDiameter(wheelDiameter),
    m_wheelBase(wheelBase),
    m_track(trackWidth),
    m_ntName(networktablename)
{
}

IChassis::CHASSIS_TYPE MecanumChassis::GetType() const
{
    return IChassis::CHASSIS_TYPE::MECANUM;
}

void MecanumChassis::Drive
(
    frc::ChassisSpeeds                     chassisSpeeds,
    IHolonomicChassis::CHASSIS_DRIVE_MODE  mode,
    IHolonomicChassis::HEADING_OPTION      headingOption
) 
{

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MecanumChassis"), string("Run Vx"), chassisSpeeds.vx.value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MecanumChassis"), string("Run Vy"), chassisSpeeds.vy.value());
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("MecanumChassis"), string("Run Omega"), chassisSpeeds.omega.value());

    auto speeds = mode == IHolonomicChassis::CHASSIS_DRIVE_MODE::FIELD_ORIENTED ? FieldDriveUtils::ConvertFieldOrientedToRobot(chassisSpeeds, m_pigeon) : chassisSpeeds;
    auto forward = speeds.vx / m_maxSpeed;
    auto strafe  = speeds.vy / m_maxSpeed;
    auto rot     = speeds.omega / m_maxAngSpeed;

    auto saturatedPower = fmax((abs(forward.value()) + abs(strafe.value()) + abs(rot.value())), 1.0);

    auto frontLeftPower  = (forward.value() + strafe.value() + rot.value()) / saturatedPower;
    auto backLeftPower   = (forward.value() - strafe.value() + rot.value()) / saturatedPower;
    auto frontRightPower = (forward.value() - strafe.value() - rot.value()) / saturatedPower;
    auto backRightPower  = (forward.value() + strafe.value() - rot.value()) / saturatedPower;

    m_leftFrontMotor.get()->Set(frontLeftPower);
    m_leftBackMotor.get()->Set(backLeftPower);
    m_rightFrontMotor.get()->Set(frontRightPower);
    m_rightBackMotor.get()->Set(backRightPower);
}

//Moves the robot
void MecanumChassis::Drive(frc::ChassisSpeeds chassisSpeeds)
{
    Drive(chassisSpeeds, IHolonomicChassis::CHASSIS_DRIVE_MODE::ROBOT_ORIENTED, IHolonomicChassis::HEADING_OPTION::MAINTAIN);
}

frc::Pose2d MecanumChassis::GetPose() const
{
    return Pose2d();
}

void MecanumChassis::ResetPose(const frc::Pose2d& pose)
{

}

void MecanumChassis::UpdateOdometry()
{
    
}

units::velocity::meters_per_second_t MecanumChassis::GetMaxSpeed() const
{
    return m_maxSpeed;
}

units::angular_velocity::radians_per_second_t MecanumChassis::GetMaxAngularSpeed() const
{
    return m_maxAngSpeed;
}

units::length::inch_t MecanumChassis::GetWheelDiameter() const
{
    return units::length::inch_t(4);
}    

units::length::inch_t MecanumChassis::GetTrack() const
{
    return m_track;
}

units::angle::degree_t MecanumChassis::GetYaw() const
{
    units::degree_t yaw{0.0}; // get from pigeon
    return yaw;
}

void MecanumChassis::SetEncodersToZero()
{
    ZeroEncoder(m_leftFrontMotor);
    ZeroEncoder(m_leftBackMotor);
    ZeroEncoder(m_rightFrontMotor);
    ZeroEncoder(m_rightBackMotor);
}
void MecanumChassis::SetTargetHeading(units::angle::degree_t targetYaw) 
{
}

void MecanumChassis::ZeroEncoder(shared_ptr<IDragonMotorController> controller)
{
    if (controller.get() != nullptr)
    {
        auto motor = controller.get()->GetSpeedController();
        auto talon = dynamic_cast<WPI_TalonSRX*>(motor.get());
        talon->SetSelectedSensorPosition(0,0);
    }
}

