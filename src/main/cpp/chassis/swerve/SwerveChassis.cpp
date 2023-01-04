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

//FRC Includes
#include <wpi/array.h>

//Team 302 Includes
#include <chassis/swerve/SwerveChassis.h>

#include <chassis/swerve/states/RobotDrive.h>
#include <chassis/swerve/states/FieldDrive.h>
#include <chassis/swerve/states/HoldDrive.h>
#include <chassis/swerve/states/StopDrive.h>
#include <chassis/swerve/states/TrajectoryDrive.h>

#include <chassis/swerve/states/orientation/MaintainHeading.h>
#include <chassis/swerve/states/orientation/SpecifiedHeading.h>
#include <chassis/swerve/states/orientation/FaceGoalHeading.h>

using std::string;

SwerveChassis::SwerveChassis
(
    std::shared_ptr<SwerveModule>                               frontLeft, 
	std::shared_ptr<SwerveModule>                               frontRight,
	std::shared_ptr<SwerveModule>                               backLeft, 
	std::shared_ptr<SwerveModule>                               backRight, 
    units::length::inch_t                                       wheelDiameter,
	units::length::inch_t                                       wheelBase,
	units::length::inch_t                                       track,
	units::velocity::meters_per_second_t                        maxSpeed,
	units::radians_per_second_t                                 maxAngularSpeed,
	units::acceleration::meters_per_second_squared_t            maxAcceleration,
	units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration,
    string                                                      networkTableName,
    string                                                      controlFileName
) : m_frontLeft(frontLeft), 
    m_frontRight(frontRight), 
    m_backLeft(backLeft), 
    m_backRight(backRight), 
    m_wheelDiameter(wheelDiameter),
    m_wheelBase(wheelBase),
    m_track(track),
    m_maxSpeed(maxSpeed),
    m_maxAngularSpeed(maxAngularSpeed), 
    m_maxAcceleration(maxAcceleration), //Not used at the moment
    m_maxAngularAcceleration(maxAngularAcceleration), //Not used at the moment
    m_swerveDriveStates(),
    m_swerveOrientation(),
    m_networkTableName(networkTableName),
    m_controlFileName(controlFileName)
{
    m_odometry = new SwerveOdometry(m_frontLeft, m_frontRight, m_backLeft, m_backRight, m_track, m_wheelBase);

    m_frontLeft->ZeroAlignModule();
    m_frontRight->ZeroAlignModule();
    m_backLeft->ZeroAlignModule();
    m_backRight->ZeroAlignModule();

    //Is this the best way to do this?
    m_swerveOrientation[SwerveEnums::MAINTAIN] = new MaintainHeading(SwerveEnums::HeadingOption::MAINTAIN);
    //..... continue

    m_currentOrientation = m_swerveOrientation[SwerveEnums::MAINTAIN];

    m_swerveDriveStates[SwerveEnums::SwerveDriveStateType::ROBOT_DRIVE] =  new RobotDrive(
                                                            SwerveEnums::SwerveDriveStateType::ROBOT_DRIVE, 
                                                            ChassisMovement{}, 
                                                            m_swerveOrientation[SwerveEnums::MAINTAIN]);

    //...... continue doing this, waiting to finish until I determine if this is right idea
}

void SwerveChassis::SetEncodersToZero()
{
    m_frontLeft.get()->SetEncodersToZero();
    m_frontRight.get()->SetEncodersToZero();
    m_backLeft.get()->SetEncodersToZero();
    m_backRight.get()->SetEncodersToZero();
}

void SwerveChassis::ZeroAlignSwerveModules()
{
    m_frontLeft->ZeroAlignModule();
    m_frontRight->ZeroAlignModule();
    m_backLeft->ZeroAlignModule();
    m_backRight->ZeroAlignModule();
}

void SwerveChassis::Drive()
{
    auto states = wpi::array(m_currentDriveState->CalcSwerveModuleStates());

    auto kinematics = m_odometry->GetSwerveDriveKinematics();
    kinematics.DesaturateWheelSpeeds(&states, m_maxSpeed);

    auto [fl,fr,bl,br] = states;

    m_frontLeft.get()->SetDesiredState(fl);
    m_frontRight.get()->SetDesiredState(fr);
    m_backLeft.get()->SetDesiredState(bl);
    m_backRight.get()->SetDesiredState(br);
}

void SwerveChassis::Drive(SwerveDriveState* targetState)
{
    m_currentDriveState = targetState;
    m_currentOrientation = targetState->GetDriveOrientation();

    m_currentDriveState->Init();
    Drive();
}

//Seems redundant to have method like this (getpose, reset pose) when we cana access odometry with GetOdometry
frc::Pose2d SwerveChassis::GetPose() const
{
    return m_odometry->GetPose();
}

void SwerveChassis::ResetPose(const frc::Pose2d& pose)
{
    m_odometry->ResetPose(pose);
}

void SwerveChassis::UpdateOdometry()
{
    m_odometry->UpdateOdometry();
}

ISwerveDriveOrientation* SwerveChassis::GetOrientation(SwerveEnums::HeadingOption orientationOption)
{
    return m_swerveOrientation[orientationOption];
}

SwerveDriveState* SwerveChassis::GetDriveState(SwerveEnums::SwerveDriveStateType stateType)
{
    return m_swerveDriveStates[stateType];
}


// need to rationalize the following from IChassis
void SwerveChassis::Drive
(
    frc::ChassisSpeeds  chassisSpeeds
)
{
  // NO-OP for now
}
void SwerveChassis::Drive
(
    frc::ChassisSpeeds  chassisSpeeds,
    CHASSIS_DRIVE_MODE  mode,
    HEADING_OPTION      headingOption
)
{
  // NO-OP for now
}
void SwerveChassis::SetTargetHeading(units::angle::degree_t targetYaw)
{
  // NO-OP for now
}
units::angle::degree_t SwerveChassis::GetYaw() const
{
    return units::angle::degree_t(0.0);
}