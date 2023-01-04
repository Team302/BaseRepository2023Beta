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

//C++ Includes
#include <memory>
#include <map>
#include <string>

//FRC Includes
#include <frc/geometry/Pose2d.h>
#include <units/angular_acceleration.h>

//Team302 Includes
#include <chassis/swerve/states/SwerveDriveState.h>
#include <chassis/swerve/SwerveOdometry.h>
#include <chassis/swerve/ISwerveDriveOrientation.h>
#include <chassis/swerve/SwerveEnums.h>
#include <chassis/swerve/SwerveModule.h>
#include <chassis/IChassis.h>



class SwerveChassis : public IChassis
{
    public:
        SwerveChassis() = delete;
        virtual ~SwerveChassis() = default;

        /// @brief Construct a SwerveChassis
        /// @param [in] std::shared_ptr<SwerveModule>           frontleft:          front left swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           frontright:         front right swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           backleft:           back left swerve module
        /// @param [in] std::shared_ptr<SwerveModule>           backright:          back right swerve module
        /// @param [in] units::length::inch_t                   wheelDiameter:      Diameter of the wheel
        /// @param [in] units::length::inch_t                   wheelBase:          distance between the front and rear wheels
        /// @param [in] units::length::inch_t                   track:              distance between the left and right wheels
        /// @param [in] units::velocity::meters_per_second_t    maxSpeed:           maximum linear speed of the chassis 
        /// @param [in] units::radians_per_second_t             maxAngularSpeed:    maximum rotation speed of the chassis 
        /// @param [in] double                                  maxAcceleration:    maximum acceleration in meters_per_second_squared
        SwerveChassis
        (
            std::shared_ptr<SwerveModule>                               frontLeft, 
			std::shared_ptr<SwerveModule>                               frontRight,
			std::shared_ptr<SwerveModule>                               backLeft, 
			std::shared_ptr<SwerveModule>                               backRight, 
            units::length::inch_t                                       wheelDiameter,
			units::length::inch_t                                       wheelBase,
			units::length::inch_t                                       track,
			units::velocity::meters_per_second_t                        maxSpeed,
			units::radians_per_second_t                                 maxAngularSpeed,
			units::acceleration::meters_per_second_squared_t            maxAcceleration,
			units::angular_acceleration::radians_per_second_squared_t   maxAngularAcceleration,
            std::string                                                 networkTableName,
            std::string                                                 controlFileName
        );

        /// @brief Runs the current SwerveDriveState
        void Drive() override;

        /// @brief Initializes the targetState, updates the current SwerveDriveState
        /// @param [in] SwerveDriveState    targetState - the new state that should be ran
        void Drive(SwerveDriveState* targetState);

        /// @brief Set all the swerve module encoders to zero
        void SetEncodersToZero();

        /// @brief Align the swerve modules to face "0" or forward
        void ZeroAlignSwerveModules();

        /// Getters

        units::length::inch_t GetWheelDiameter() const {return m_wheelDiameter; }  
        units::length::inch_t GetWheelBase() const {return m_wheelBase; }  
        units::length::inch_t GetTrack() const {return m_track;}
        units::velocity::meters_per_second_t GetMaxSpeed() const {return m_maxSpeed;}
        units::radians_per_second_t GetMaxAngularSpeed() const {return m_maxAngularSpeed;}
        units::acceleration::meters_per_second_squared_t GetMaxAcceleration() const { return m_maxAcceleration; }
        units::angular_acceleration::radians_per_second_squared_t GetMaxAngularAcceleration() const { return m_maxAngularAcceleration; }
        std::shared_ptr<SwerveModule> GetFrontLeft() const { return m_frontLeft;}
        std::shared_ptr<SwerveModule> GetFrontRight() const { return m_frontRight;}
        std::shared_ptr<SwerveModule> GetBackLeft() const { return m_backLeft;}
        std::shared_ptr<SwerveModule> GetBackRight() const { return m_backRight;};

        inline IChassis::CHASSIS_TYPE GetType() const override {return IChassis::CHASSIS_TYPE::SWERVE;};
        inline void Initialize() override {};

        /// @brief Returns the current SwerveDriveState
        /// @return SwerveDriveState* - current SwerveDriveState
        SwerveDriveState* GetCurrentDriveState() const {return m_currentDriveState;};

        /// @brief Get the specified SwerveDriveState
        /// @return SwerveDriveState* - specified state
        SwerveDriveState* GetDriveState(SwerveEnums::SwerveDriveStateType stateType);

        /// @brief Get current estimated chassis position as Pose2d
        /// @return frc::Pose2d - current chassis position
        frc::Pose2d GetPose() const;

        /// @brief Get SwerveOdometry
        /// @return SwerveOdometry* - the chassis' odometry
        SwerveOdometry* GetOdometry() const override {return m_odometry;};

        /// @brief Set the current chassis position to the target pose
        /// @param [in] frc::Pose2d pose - target pose
        void ResetPose(const frc::Pose2d& pose) override;

        /// @brief Update current estimated chassis position based on encoders and sensors
        void UpdateOdometry();

        /// @brief Get the current chassis orientation "state"
        /// @return ISwerveDriveOrientation* - current orientation
        ISwerveDriveOrientation* GetCurrentOrientation() const {return m_currentOrientation;};

        /// @brief Get the specified chassis orientation "state"
        /// @return ISwerveDriveOrientation* - specified orientation
        ISwerveDriveOrientation* GetOrientation(SwerveEnums::HeadingOption orientationOption);

        // need to rationalize the following from IChassis
        void Drive
        (
            frc::ChassisSpeeds  chassisSpeeds
        ) override;
        void Drive
        (
            frc::ChassisSpeeds  chassisSpeeds,
            CHASSIS_DRIVE_MODE  mode,
            HEADING_OPTION      headingOption
        ) override;
        void SetTargetHeading(units::angle::degree_t targetYaw) override;
        units::angle::degree_t GetYaw() const override;

    private:
        std::shared_ptr<SwerveModule>                                       m_frontLeft;
        std::shared_ptr<SwerveModule>                                       m_frontRight;
        std::shared_ptr<SwerveModule>                                       m_backLeft;
        std::shared_ptr<SwerveModule>                                       m_backRight;

        units::length::inch_t                                               m_wheelDiameter;       
        units::length::inch_t                                               m_wheelBase;       
        units::length::inch_t                                               m_track;
        units::velocity::meters_per_second_t                                m_maxSpeed;
        units::radians_per_second_t                                         m_maxAngularSpeed;
        units::acceleration::meters_per_second_squared_t                    m_maxAcceleration;
        units::angular_acceleration::radians_per_second_squared_t           m_maxAngularAcceleration;

        //Dont know if this and orientation should be vectors or maps, b/c of GetDriveState: I feel like they should be maps
        //std::vector<SwerveDriveState*>          m_swerveDriveStates;
        SwerveDriveState*                                                   m_currentDriveState;

        std::map<SwerveEnums::SwerveDriveStateType, SwerveDriveState*> m_swerveDriveStates;

        //std::vector<ISwerveDriveOrientation*>   m_swerveOrientation;
        std::map<SwerveEnums::HeadingOption, ISwerveDriveOrientation*>      m_swerveOrientation;
        ISwerveDriveOrientation*                                            m_currentOrientation;

        SwerveOdometry*                                                     m_odometry;
        std::string                                                         m_networkTableName;
        std::string                                                         m_controlFileName;
};