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
#include <hw/DragonSolenoid.h>

class Mech1Solenoid : public Mech
{
    public:
        /// @brief Create a generic mechanism wiht 1 solenoid 
        /// @param [in] std::shared_ptr<DragonSolenoid> solenoid used by this mechanism
         Mech1Solenoid
        (
            MechanismTypes::MECHANISM_TYPE              type,
            std::string                                 controlFileName,
            std::string                                 networkTableName,
            std::shared_ptr<DragonSolenoid>             solenoid
        );

        Mech1Solenoid() = delete;
        virtual ~Mech1Solenoid() = default;

        /// @brief      Activate/deactivate pneumatic solenoid
        /// @param [in] bool - true == extend, false == retract
        /// @return     void 
        void ActivateSolenoid
        (
            bool     activate
        );

        /// @brief      Check if the pneumatic solenoid is activated
        /// @return     bool - true == extended, false == retracted
        bool IsSolenoidActivated() const;

        /// @brief log data to the network table if it is activated and time period has past
        void LogHardwareInformation();

    private:
        std::shared_ptr<DragonSolenoid>             m_solenoid;

};
