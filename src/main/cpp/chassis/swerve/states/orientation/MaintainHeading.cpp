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
#include <chassis/swerve/states/orientation/MaintainHeading.h>
#include <chassis/swerve/SwerveOdometry.h>

MaintainHeading::MaintainHeading(SwerveEnums::HeadingOption headingOption
) : ISwerveDriveOrientation(headingOption)
{
    
}

void MaintainHeading::UpdateChassisSpeeds(ChassisMovement& chassisMovement)
{
    units::angular_velocity::degrees_per_second_t correction = units::angular_velocity::degrees_per_second_t(0.0);

    units::meters_per_second_t xspeed = chassisMovement.chassisSpeeds.vx;
    units::meters_per_second_t yspeed = chassisMovement.chassisSpeeds.vy;
    units::radians_per_second_t rot = chassisMovement.chassisSpeeds.omega;

    if(abs(rot.to<double>()) == 0.0)
    {
        rot = units::radians_per_second_t(0.0);
        if (abs(xspeed.to<double>()) > 0.0 || abs(yspeed.to<double>() > 0.0))
        {
            correction = CalcHeadingCorrection(m_storedYaw, m_kPMaintainHeadingControl);
        }
    }
    else
    {
        m_storedYaw = SwerveOdometry::GetInstance()->GetPose().Rotation().Degrees();
    }

    rot -= correction;
}