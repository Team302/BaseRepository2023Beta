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
#include <chassis/swerve/states/orientation/FaceGoalHeading.h>
#include <hw/factories/LimelightFactory.h>

#include <chassis/swerve/SwerveOdometry.h>

FaceGoalHeading::FaceGoalHeading(SwerveEnums::HeadingOption headingOption
) : ISwerveDriveOrientation(headingOption),
    m_limelight(LimelightFactory::GetLimelightFactory()->GetLimelight())
{

}

void FaceGoalHeading::UpdateChassisSpeeds(ChassisMovement& chassisMovement)
{
    units::angular_velocity::radians_per_second_t rot = chassisMovement.chassisSpeeds.omega;

    if(abs(m_limelight->GetTargetHorizontalOffset().to<double>()) < 1.0 && m_limelight->HasTarget())
    {
        //Hold position
    }
    else if (m_limelight != nullptr && m_limelight->HasTarget())
    { 
        double rotCorrection = abs(m_limelight->GetTargetHorizontalOffset().to<double>()) > 10.0 ? m_kPGoalHeadingControl : m_kPGoalHeadingControl*2.0;
        rot += (m_limelight->GetTargetHorizontalOffset())/1_s*rotCorrection; 
    }
    else
    {
        auto targetAngle = units::angle::degree_t(m_targetFinder.GetTargetAngleD(SwerveOdometry::GetInstance()->GetPose()));
        rot -= CalcHeadingCorrection(targetAngle,m_kPGoalHeadingControl);
    }
}