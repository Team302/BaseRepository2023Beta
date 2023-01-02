
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

#include <frc/BuiltInAccelerometer.h>

class DragonRoboRioAccelXDownYRight : public frc::BuiltInAccelerometer
{
	public:
		DragonRoboRioAccelXDownYRight() = default;
		virtual ~DragonRoboRioAccelXDownYRight() = default;

		 /// @return The acceleration of the roboRIO along the robot X axis (forward) in g-forces
		inline double GetX() override {return -1.0 * BuiltInAccelerometer::GetZ();}

		 /// @return The acceleration of the roboRIO along the robot Y axis (left) in g-forces
		inline double GetY() override {return -1.0 * BuiltInAccelerometer::GetY();}

		 /// @return The acceleration of the roboRIO along the robot Z axis (up) in g-forces
		inline double GetZ() override {return -1.0 * BuiltInAccelerometer::GetX();}
};



