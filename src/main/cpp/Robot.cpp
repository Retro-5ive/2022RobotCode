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
ctre::phoenix::motorcontrol::can::WPI_TalonSRX left{3};  //left motor is a talon
ctre::phoenix::motorcontrol::can::WPI_TalonSRX right{6};   //right motor is a talon

frc::DifferentialDrive tankDrive{left, right};    //make left side and right side into one drive - tank drive


//Talon mechanism motors initialization
ctre::phoenix::motorcontrol::can::WPI_TalonSRX intake{1};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX shooter{0};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX climber{2};


//SparkMax and Reg spark initialization
frc::PWMSparkMax index2{8};  //the number in the bracket is the port it's connected to in Roborio


//Joysticks instantiation
frc::Joystick drivePad1{0};    //port 0 is a joystick for drive
frc::Joystick drivePad2{1};     //port 1 is joystick for drive
frc::GenericHID mechPad{2};       //port 2 is a gamepad for mechanisms


public:
void RobotInit() override {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    left.SetInverted(true);
  }

void TeleopPeriodic() override {

    // Drive with tank style using drivePad
    tankDrive.TankDrive(drivePad1.GetRawAxis(1), drivePad2.GetRawAxis(1));  //axis 1 and 3 from drivePad are to gauge the drive

    //attach mechanisms to mechpad
    index2.Set(mechPad.GetRawButton(1));
    shooterFunction();
    climberFunction();
    intakeFunction();
  }



void shooterFunction(){
  if(mechPad.GetRawButton(8)){

      shooter.Set(.25); //intake

  }else{

    shooter.Set(0);
      
  }
}

void climberFunction(){
    if(mechPad.GetPOV(0)){

    climber.Set(1);

  }else if(mechPad.GetPOV(180)){

    climber.Set(-1);

  }else{

    climber.Set(0);
  }
}

void intakeFunction(){
  if(mechPad.GetRawButton(6)){

    intake.Set(.75); //intake

  }else if(mechPad.GetRawButton(5)){

    intake.Set(-.75);   //outtake

  }else{

    intake.Set(0);
    
  }
}

};






#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif


