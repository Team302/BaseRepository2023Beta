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

//Team302 Includes
#include <chassis/swerve/states/TrajectoryDrive.h>

TrajectoryDrive::TrajectoryDrive(RobotDrive robotDrive
) : RobotDrive(robotDrive.GetStateType(), robotDrive.GetChassisMovement(), robotDrive.GetDriveOrientation()),
    m_trajectory(m_chassisMovement.trajectory),
    m_robotDrive(robotDrive),
    m_holonomicController(frc2::PIDController{1.5, 0, 0},
                          frc2::PIDController{1.5, 0, 0},
                          frc::ProfiledPIDController<units::radian>{0.1, 0, 0,
                          frc::TrapezoidProfile<units::radian>::Constraints{0_rad_per_s, 0_rad_per_s / 1_s}}),
    m_desiredState(),
    m_trajectoryStates(m_trajectory.States()),
    m_timer(std::make_unique<frc::Timer>())
{

}

void TrajectoryDrive::Init()
{
    //Clear m_trajectoryStates in case it holds onto a previous trajectory
    m_trajectoryStates.clear();


    if (!m_trajectoryStates.empty()) // only go if path name found
    {
        //Desired state is first state in trajectory
        m_desiredState = m_trajectoryStates.front(); //m_desiredState is the first state, or starting position

        m_timer.get()->Reset(); //Restarts and starts timer
        m_timer.get()->Start();
    }
}

std::array<frc::SwerveModuleState, 4> TrajectoryDrive::CalcSwerveModuleStates()
{
    if (!m_trajectoryStates.empty()) //If we have a path parsed / have states to run
    {
        // calculate where we are and where we want to be
        CalcCurrentAndDesiredStates();

        // Use the controller to calculate the chassis speeds for getting there
        frc::ChassisSpeeds refChassisSpeeds;
        refChassisSpeeds = m_holonomicController.Calculate(m_chassis->GetOdometry()->GetPose(), 
                                                          m_desiredState, 
                                                          m_desiredState.pose.Rotation());
        //Set chassisMovement speeds that will be used by RobotDrive
        m_chassisMovement.chassisSpeeds = refChassisSpeeds;
        return m_robotDrive.CalcSwerveModuleStates();

    }
    else //If we don't have states to run, don't move the robot
    {
        //Create 0 speed frc::ChassisSpeeds
        frc::ChassisSpeeds speeds;
        speeds.vx = 0_mps;
        speeds.vy = 0_mps;
        speeds.omega = units::angular_velocity::radians_per_second_t(0);
        
        //Set chassisMovement speeds that will be used by RobotDrive
        m_chassisMovement.chassisSpeeds = speeds;
        return m_robotDrive.CalcSwerveModuleStates();
    }
}

void TrajectoryDrive::CalcCurrentAndDesiredStates()
{
    //Get current time
    auto sampleTime = units::time::second_t(m_timer.get()->Get());
    //Set desired state to the state at current time
    m_desiredState = m_trajectory.Sample(sampleTime);
}