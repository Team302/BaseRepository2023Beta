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
#include <string>

// FRC includes
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include <frc/kinematics/ChassisSpeeds.h>

// Team 302 Includes
#include <chassis/holonomic/HolonomicDrive.h>
#include <chassis/IChassis.h>
#include <hw/DragonPigeon.h>
#include <gamepad/IDragonGamePad.h>
#include <TeleopControl.h>
#include <State.h>
#include <chassis/ChassisFactory.h>
#include <hw/factories/PigeonFactory.h>
#include <utils/Logger.h>

using namespace std;
using namespace frc;

/// @brief initialize the object and validate the necessary items are not nullptrs
HolonomicDrive::HolonomicDrive() : State(string("HolonomicDrive"), -1),
                                   m_chassis(ChassisFactory::GetChassisFactory()->GetIChassis()),
                                   m_controller(TeleopControl::GetInstance())
{
    if (m_controller == nullptr)
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::ERROR, string("HolonomicDrive"), string("Constructor"), string("TeleopControl is nullptr"));
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
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Run"), string("begin"));
    auto controller = GetController();
    if (controller != nullptr && m_chassis != nullptr)
    {
        IChassis::CHASSIS_DRIVE_MODE mode = IChassis::CHASSIS_DRIVE_MODE::FIELD_ORIENTED;
        IChassis::HEADING_OPTION headingOpt = IChassis::HEADING_OPTION::MAINTAIN;
        if (controller->IsButtonPressed(TeleopControl::FINDTARGET))
        {
            headingOpt = IChassis::HEADING_OPTION::TOWARD_GOAL;
        }                                       
        else if (controller->IsButtonPressed(TeleopControl::DRIVE_TO_SHOOTING_SPOT))
        {
            headingOpt = IChassis::HEADING_OPTION::TOWARD_GOAL_DRIVE;
        }
        
        if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::REZERO_PIGEON))
        {
            auto factory = PigeonFactory::GetFactory();
            auto m_pigeon = factory->GetPigeon(DragonPigeon::PIGEON_USAGE::CENTER_OF_ROBOT);
            m_pigeon->ReZeroPigeon(0.0, 0.0);
            //m_chassis->ZeroAlignSwerveModules();
            //m_chassis->ReZero();
        }

        //if (controller->IsButtonPressed(TeleopControl::FUNCTION_IDENTIFIER::HOLD_POSITION))
        //{
            //m_chassis.get()->DriveHoldPosition();
        //}

        auto maxSpeed = m_chassis->GetMaxSpeed();
        auto maxAngSpeed = m_chassis->GetMaxAngularSpeed();

        auto forward = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_FORWARD);
        auto strafe = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_STRAFE);
        auto rotate = controller->GetAxisValue(TeleopControl::FUNCTION_IDENTIFIER::HOLONOMIC_DRIVE_ROTATE);

        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("axis read"));

        ChassisSpeeds speeds{forward*maxSpeed, strafe*maxSpeed, rotate*maxAngSpeed};
        m_chassis->Drive(speeds, mode, headingOpt);
    }
    else
    {
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("HolonomicDrive"), string("Run"), string("nullptr"));
    }
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, string("ArrivedAt"), string("HolonomicDrive::Run"), string("end"));   
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