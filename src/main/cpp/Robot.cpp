// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMTalonFX.h>
#include <frc/motorcontrol/PWMMotorController.h>
#include "ctre/phoenix6/CANBus.hpp"

#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"

/**
 * 
 * Runs the motors with tank steering and an Xbox controller.
 */
class Robot : public frc::TimedRobot {
  ctre::phoenix6::hardware::TalonFX m1{2}; 
  ctre::phoenix6::hardware::TalonFX m2{3}; 
  ctre::phoenix6::hardware::TalonFX m3{4}; 
  ctre::phoenix6::hardware::TalonFX m4{5}; 

  


  frc::DifferentialDrive m_robotDrive{
      [&](double output) { m1.Set(output); },
      [&](double output) { m2.Set(output); },
      
    };
  frc::XboxController m_driverController{0};

 public:
  Robot() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m1);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m2);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m3);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &m4);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m3.SetControl(ctre::phoenix6::controls::Follower(2, ctre::phoenix6::signals::MotorAlignmentValue::Aligned));
    m4.SetControl(ctre::phoenix6::controls::Follower(3, ctre::phoenix6::signals::MotorAlignmentValue::Aligned));

  ctre::phoenix6::configs::MotorOutputConfigs outputConfig1{};
  outputConfig1.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;
  m1.GetConfigurator().Apply(outputConfig1);


  ctre::phoenix6::configs::MotorOutputConfigs outputConfig2{};
  outputConfig2.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
  m2.GetConfigurator().Apply(outputConfig2);

  
  }

  void TeleopPeriodic() override {
    // Drive with tank style
    m_robotDrive.TankDrive(-m_driverController.GetLeftY(),
                           -m_driverController.GetRightY());
  }
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
