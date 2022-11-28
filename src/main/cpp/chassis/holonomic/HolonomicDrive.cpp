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
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/kinematics/ChassisSpeeds.h>

// Team 302 Includes
#include <chassis/holonomic/HolonomicDrive.h>
#include <chassis/IChassis.h>
#include <chassis/IHolonomicChassis.h>
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
HolonomicDrive::HolonomicDrive() : IState(),
                                 m_chassis(ChassisFactory::GetChassisFactory()->GetMecanumChassis()),
                                 m_holonomicChassis(ChassisFactory::GetChassisFactory()->GetHolonomicChassis()),
                                 m_controller(TeleopControl::GetInstance())
{
    if (m_controller == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("HolonomicDrive"), string("Constructor"), string("TeleopControl is nullptr"));
    }

    if (m_holonomicChassis == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("HolonomicDrive"), string("Constructor"), string("Chassis is nullptr"));
    }
}

/// @brief initialize the profiles for the various gamepad inputs
/// @return void
void HolonomicDrive::Init()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Init"), string("arrived"));   
    auto controller = GetController();
    if (controller != nullptr)
    {
        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_FORWARD, IDragonGamePad::AXIS_PROFILE::CUBED);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_FORWARD, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_FORWARD, -0.6);

        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_STRAFE, IDragonGamePad::AXIS_PROFILE::CUBED);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_STRAFE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_STRAFE, -0.6);

        controller->SetAxisProfile(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_ROTATE, IDragonGamePad::AXIS_PROFILE::CUBED);
        controller->SetDeadBand(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_ROTATE, IDragonGamePad::AXIS_DEADBAND::APPLY_STANDARD_DEADBAND);
        controller->SetAxisScaleFactor(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_ROTATE, 0.5);
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Init"), string("end"));   
}

/// @brief calculate the output for the wheels on the chassis from the throttle and steer components
/// @return void
void HolonomicDrive::Run()
{
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("begin"));
    auto controller = GetController();
    if (controller != nullptr && m_holonomicChassis != nullptr && m_chassis != nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("not nullptr"));

        IHolonomicChassis::CHASSIS_DRIVE_MODE mode = IHolonomicChassis::CHASSIS_DRIVE_MODE::FIELD_ORIENTED;
        IHolonomicChassis::HEADING_OPTION headingOpt = IHolonomicChassis::HEADING_OPTION::MAINTAIN;

        auto maxSpeed = m_chassis->GetMaxSpeed();
        auto maxAngSpeed = m_holonomicChassis->GetMaxAngularSpeed();

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("about to read joysticks"), string("xxx"));
        auto forward = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_FORWARD);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("forward"), forward);
        auto strafe = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_STRAFE);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("strafe"), strafe);
        auto rotate = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_ROTATE);
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("rotate"), rotate);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("axis read"));

        ChassisSpeeds speeds{forward*maxSpeed, strafe*maxSpeed, rotate*maxAngSpeed};
        //speeds.vx = forward * maxSpeed;
        //speeds.vy =  strafe * maxSpeed;
        //speeds.omega = rotate * maxAngSpeed;
        
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run Vx"), speeds.vx.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run Vy"), speeds.vy.value());
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run Omega"), speeds.omega.value());

        m_holonomicChassis->Drive(speeds, mode, headingOpt);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("nullptr"));
    }
}

void HolonomicDrive::Exit()
{
}

/// @brief indicates that we are not at our target
/// @return bool
bool HolonomicDrive::AtTarget() const
{
    return false;
}