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

// C++ Includes
#include <algorithm>
#include <memory>

// FRC includes
#include <frc/drive/Vector2d.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>

// Team 302 Includes
#include <chassis/differential/ArcadeDrive.h>
#include <hw/DragonPigeon.h>
#include <gamepad/IDragonGamePad.h>
#include <TeleopControl.h>
#include <mechanisms/base/IState.h>
#include <chassis/ChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>

using namespace std;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
ArcadeDrive::ArcadeDrive() : IState(),
                             m_chassis(ChassisFactory::GetChassisFactory()->GetDifferentialChassis()),
                             m_controller(TeleopControl::GetInstance())
{
    if (m_controller == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("ArcadeDrive"), string("Constructor"), string("TeleopControl is nullptr"));
    }

    if (m_chassis.get() == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("ArcadeDrive"), string("Constructor"), string("Chassis is nullptr"));
    }
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void ArcadeDrive::Init()
{
    auto controller = GetController();
    if (controller != nullptr)
    {
        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::ARCADE_STEER, IDragonGamePad::AXIS_PROFILE::CUBED);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::ARCADE_STEER, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::ARCADE_THROTTLE, IDragonGamePad::AXIS_PROFILE::CUBED);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::ARCADE_THROTTLE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
    }
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void ArcadeDrive::Run()
{
    if (m_chassis != nullptr && m_controller != nullptr)
    {
        auto throttle = m_controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::ARCADE_THROTTLE);
        auto steer = m_controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::ARCADE_STEER);

        frc::ChassisSpeeds speeds;
        speeds.vx = throttle * m_chassis->GetMaxSpeed();
        speeds.vy = 0_mps; //units::velocity::meters_per_second_t(0)
        speeds.omega = steer * m_chassis->GetMaxAngularSpeed();
        m_chassis->Drive(speeds);
    }
}

void ArcadeDrive::Exit()
{
}

/// @brief indicates that we are not at our target
/// @return bool
bool ArcadeDrive::AtTarget() const
{
    return false;
}