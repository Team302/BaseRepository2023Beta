
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

#include <string>

#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/MecanumDriveKinematics.h>
#include <frc/kinematics/MecanumDriveOdometry.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include <hw/factories/PigeonFactory.h>
#include <hw/DragonPigeon.h>

#include <chassis/mecanum/MecanumChassis.h>

#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>

using namespace std;
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
    m_drive(new frc::MecanumDrive(*(leftFrontMotor.get()->GetSpeedController().get()), 
                                  *(leftBackMotor.get()->GetSpeedController().get()),
                                  *(rightFrontMotor.get()->GetSpeedController().get()),
                                  *(rightBackMotor.get()->GetSpeedController().get()))),
    m_kinematics(new frc::MecanumDriveKinematics(frc::Translation2d(wheelBase/2.0, trackWidth/2.0), 
                                                 frc::Translation2d(wheelBase/2.0, -1.0*trackWidth/2.0),
                                                 frc::Translation2d(-1.0*wheelBase/2.0, 1.0*trackWidth/2.0),
                                                 frc::Translation2d(-1.0*wheelBase/2.0, -1.0*trackWidth/2.0))),
    m_ntName(networktablename)
{
}

IChassis::CHASSIS_TYPE MecanumChassis::GetType() const
{
    return IChassis::CHASSIS_TYPE::MECANUM;
}

void MecanumChassis::Drive
(
    frc::ChassisSpeeds            chassisSpeeds,
    IChassis::CHASSIS_DRIVE_MODE  mode,
    IChassis::HEADING_OPTION      headingOption
) 
{
    //auto wheels = m_kinematics->ToWheelSpeeds(chassisSpeeds);
    //wheels.Desaturate(m_maxSpeed);

    if (m_drive != nullptr)
    {
        if (mode == IChassis::CHASSIS_DRIVE_MODE::FIELD_ORIENTED && m_pigeon != nullptr)
        {
            m_drive->DriveCartesian(chassisSpeeds.vx(), chassisSpeeds.vy(), chassisSpeeds.omega(), m_pigeon->GetYaw());
        }
        else
        {
            m_drive->DriveCartesian(chassisSpeeds.vx(), chassisSpeeds.vy(), chassisSpeeds.omega());
        }
    }
}

//Moves the robot
void MecanumChassis::Drive(frc::ChassisSpeeds chassisSpeeds)
{
    Drive(chassisSpeeds, IChassis::CHASSIS_DRIVE_MODE::ROBOT_ORIENTED, IChassis::HEADING_OPTION::MAINTAIN);
}

frc::Pose2d MecanumChassis::GetPose() const
{
    return frc::Pose2d();
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
        auto fx = dynamic_cast<WPI_TalonFX*>(motor.get());
        auto driveMotorSensors = fx->GetSensorCollection();
        driveMotorSensors.SetIntegratedSensorPosition(0, 0);
    }
}

