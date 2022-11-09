
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
#include <memory>
#include <string>

// Team 302 includes
#include <mechanisms/base/Mech.h>
#include <mechanisms/MechanismTypes.h>

// forward declares
class ControlData;
class IDragonMotorController;

class Mech2IndMotors : public Mech
{
	public:
        /// @brief Create a generic mechanism wiht 2 independent motors 
        /// @param [in] MechanismTypes::MECHANISM_TYPE the type of mechansim
        /// @param [in] std::string the name of the file that will set control parameters for this mechanism
        /// @param [in] std::string the name of the network table for logging information
        /// @param [in] std::shared_ptr<IDragonMotorController> primary motor used by this mechanism
        /// @param [in] std::shared_ptr<IDragonMotorController> secondary motor used by this mechanism
         Mech2IndMotors
        (
            MechanismTypes::MECHANISM_TYPE              type,
            std::string                                 controlFileName,
            std::string                                 networkTableName,
            std::shared_ptr<IDragonMotorController>     spinMotor,
            std::shared_ptr<IDragonMotorController>     liftMotor
        );
	    Mech2IndMotors() = delete;
	    ~Mech2IndMotors() = default;

        /// @brief log data to the network table if it is activated and time period has past
        void LogHardwareInformation() override;

        /// @brief update the output to the mechanism using the current controller and target value(s)
        /// @return void 
        void Update();

        void UpdateTargets
        (
            double      primary,
            double      secondary
        );

        /// @brief  Return the current position of the primary motor in the mechanism.  The value is in inches or degrees.
        /// @return double	position in inches (translating mechanisms) or degrees (rotating mechanisms)
        double GetPrimaryPosition() const;

        /// @brief  Return the current position of the secondary motor in the mechanism.  The value is in inches or degrees.
        /// @return double	position in inches (translating mechanisms) or degrees (rotating mechanisms)
        double GetSecondaryPosition() const;

        /// @brief  Get the current speed of the primary motor in the mechanism.  The value is in inches per second or degrees per second.
        /// @return double	speed in inches/second (translating mechanisms) or degrees/second (rotating mechanisms)
        double GetPrimarySpeed() const;

        /// @brief  Get the current speed of the secondary motor in the mechanism.  The value is in inches per second or degrees per second.
        /// @return double	speed in inches/second (translating mechanisms) or degrees/second (rotating mechanisms)
        double GetSecondarySpeed() const;

        /// @brief  Set the control constants (e.g. PIDF values).
        /// @param [in] ControlData*                                   pid:  the control constants
        /// @return void
        void SetControlConstants
        (
            int                                         slot,
            ControlData*                                pid                 
        );
        void SetSecondaryControlConstants
        (
            int                                         slot,
            ControlData*                                pid                 
        );

        double GetPrimaryTarget() const { return m_primaryTarget; }
        double GetSecondaryTarget() const { return m_secondaryTarget; }

        inline std::shared_ptr<IDragonMotorController> GetPrimaryMotor() const {return m_primary;};
        inline std::shared_ptr<IDragonMotorController> GetSecondaryMotor() const {return m_secondary;};

    private: 
        std::shared_ptr<IDragonMotorController>     m_primary;
        std::shared_ptr<IDragonMotorController>     m_secondary;
        double                                      m_primaryTarget;
        double                                      m_secondaryTarget;
        
};



