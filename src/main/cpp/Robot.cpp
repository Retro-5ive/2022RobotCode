// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/GenericHID.h>
#include <frc/TimedRobot.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Joystick.h> 
#include "rev/SparkMaxPIDController.h"
#include <frc/motorcontrol/PWMSparkMax.h>



class Robot : public frc::TimedRobot {      


//Talon drive motors initialization and groupings
ctre::phoenix::motorcontrol::can::WPI_TalonSRX leftf{3};  //left back motor is a talon
frc::SpeedControllerGroup leftDrive{leftf};          //combine front and back motors

ctre::phoenix::motorcontrol::can::WPI_TalonSRX rightf{6};   //right front motor is a talon

frc::SpeedControllerGroup rightDrive{rightf};       //combine front and back motors

frc::DifferentialDrive tankDrive{leftDrive, rightDrive};    //make left side and right side into one drive - tank drive


//Talon mechanism motors initialization
ctre::phoenix::motorcontrol::can::WPI_TalonSRX intake{1};
//ctre::phoenix::motorcontrol::can::WPI_TalonSRX outtake{3};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX shooter{0};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX climberDown{2};


//SparkMax and Reg spark initialization
frc::PWMSparkMax index2{8};
//frc::PWMSparkMax climberUp{10};

//Joysticks instantiation
frc::Joystick drivePad{0};      //port 0 is a gamepad for drive
frc::GenericHID mechPad{1};       //port 1 is a gamepad for mechanisms

public:
void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftDrive.SetInverted(true);
   // shooter.Set(PercentOutput, 1);
  }

void TeleopPeriodic() override {

    // Drive with tank style using drivePad
    tankDrive.TankDrive(drivePad.GetRawAxis(1), drivePad.GetRawAxis(3));  //axis 1 and 3 from drivePad are to gauge the drive

    //attach mechanisms to mechpad
    //climberUp.Set(mechPad.GetPOV(0));
    //climberDown.Set(mechPad.GetPOV(180));
    index2.Set(mechPad.GetRawButton(1));
    intake.Set(mechPad.GetRawButton(6));
    //outtake.Set(mechPad.GetRawButton(5));
    shooter.Set(mechPad.GetRawButton(8));

  }
};


#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif


