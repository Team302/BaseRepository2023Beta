
//====================================================================================================================================================
/// Copyright 2022 Lake Orion Robotics FIRST Team 302 
///
/// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
/// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
/// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
/// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================

#pragma once
#include <string>

#include <State.h>

class Mech2Servos;

class Mech2ServosState : public State
{
    public:

        Mech2ServosState
        (
            Mech2Servos*                    mechanism,
            std::string                     stateName,
            int                             stateId,
            double                          target,
            double                          target2
        );
        Mech2ServosState() = delete;
        ~Mech2ServosState() = default;

        void Init() override;
        void Run() override;
        void Exit() override;
        bool AtTarget() const override;

        void LogInformation() const override;
        
        double GetTarget() const {return m_target;}
        double GetTarget2() const {return m_target2;}

    private:

        Mech2Servos*                    m_mechanism;
        double                          m_target;
        double                          m_target2;
};
