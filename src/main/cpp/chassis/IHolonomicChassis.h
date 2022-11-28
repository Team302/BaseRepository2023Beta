
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

// C++ Includes

// FRC includes
#include <units/angular_velocity.h>

// Team 302 includes

// Third Party Includes


namespace frc
{
    struct ChassisSpeeds;
}


///	 @interface IHolonomicChassis
///  @brief	    Interface for differential drives
class IHolonomicChassis
{
	public:
        enum CHASSIS_DRIVE_MODE
        {
            ROBOT_ORIENTED,
            FIELD_ORIENTED,
            POLAR_DRIVE
        };

        enum HEADING_OPTION
        {
            DEFAULT,
            MAINTAIN,
            POLAR_HEADING,
            TOWARD_GOAL,
            TOWARD_GOAL_DRIVE,
            TOWARD_GOAL_LAUNCHPAD,
            SPECIFIED_ANGLE,
            LEFT_INTAKE_TOWARD_BALL,
            RIGHT_INTAKE_TOWARD_BALL
        };


        /// @brief      Run chassis 
        /// @returns    void
        virtual void Drive
        (
            frc::ChassisSpeeds  chassisSpeeds,
            CHASSIS_DRIVE_MODE  mode,
            HEADING_OPTION      headingOption
        ) = 0;
        

        virtual void SetTargetHeading(units::angle::degree_t targetYaw) = 0;

	    IHolonomicChassis() = default;
	    virtual ~IHolonomicChassis() = default;
};



