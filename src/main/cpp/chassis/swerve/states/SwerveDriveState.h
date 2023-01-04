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

#pragma once

//FRC Includes
#include <frc/kinematics/SwerveModuleState.h>

//Team302 Includes
#include <chassis/swerve/ISwerveDriveOrientation.h>
#include <chassis/swerve/ChassisMovement.h>
#include <chassis/swerve/SwerveEnums.h>



class SwerveDriveState
{
    public:
        SwerveDriveState(SwerveEnums::SwerveDriveStateType stateType, ChassisMovement chassisMovement, ISwerveDriveOrientation* swerveOrientation);

        /// @brief Get the state type
        /// @return SwerveDriveStateType - type of state
        SwerveEnums::SwerveDriveStateType GetStateType() const {return m_stateType;};

        /// @brief Get the current ChassisMovement object
        /// @return ChassisMovement - current ChassisMovement
        ChassisMovement GetChassisMovement() const {return m_chassisMovement;};

        /// @brief Get the current swerve drive orientation
        /// @return ISwerveDriveOrientation - current orientation
        ISwerveDriveOrientation* GetDriveOrientation() const {return m_orientation;};

        /// @brief Initialize the state
        void virtual Init() = 0;
        
        /// @brief Calculate the swerve module states based on chassis movement and orientation option
        /// @return std::array<frc::SwerveModuleState*, 4> - 4 calculated swerve module states
        virtual  std::array<frc::SwerveModuleState, 4> CalcSwerveModuleStates() = 0;

        /// @brief Update the ChassisMovement for the state
        /// @param [in] ChassisMovement chassisMovement - new chassisMovement
        void UpdateChassisMovement(ChassisMovement chassisMovement);

        /// @brief Update the orientation option for the state
        /// @param [in] ISwerveDriveOrientation orientation - new orientation
        void UpdateOrientationOption(ISwerveDriveOrientation* orientation);

    protected:
        SwerveEnums::SwerveDriveStateType        m_stateType;
        ChassisMovement             m_chassisMovement;
        ISwerveDriveOrientation*     m_orientation;
};