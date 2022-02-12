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
#include <frc/Timer.h>


class Robot : public frc::TimedRobot {      

double shooterSpeed = 0.25;

//Talon drive motors initialization and groupings
ctre::phoenix::motorcontrol::can::WPI_TalonSRX left{3};  //left motor is a talon
ctre::phoenix::motorcontrol::can::WPI_TalonSRX right{6};   //right motor is a talon

frc::DifferentialDrive tankDrive{left, right};    //make left side and right side into one drive - tank drive


//Talon mechanism motors initialization
ctre::phoenix::motorcontrol::can::WPI_TalonSRX intake{1};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX shooter{0};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX climber{2};


//SparkMax and Reg spark initialization
frc::PWMSparkMax index{8};  //the number in the bracket is the port it's connected to in Roborio


//Joysticks instantiation
frc::Joystick drivePad1{0};    //port 0 is a joystick for drive
frc::Joystick drivePad2{1};     //port 1 is joystick for drive
frc::GenericHID mechPad{2};       //port 2 is a gamepad for mechanisms

//timer
 frc::Timer timer;



public:

void RobotInit() override { //This runs on initialization of the robot during teleop

    left.SetInverted(true);
    timer.Reset();
    timer.Start();
    
  }


void AutonomousPeriodic() override{ // This is called periodically while the robot is in autonomous mode.
  timer.GetFPGATimestamp();

  shooter.Set(shooterSpeed);    //shooter is running through entire autonomous period
  
  if(timer.HasElapsed(units::second_t(2))){

    index.Set(1.0);   //2 seconds later run index to shoot ball

  }else if(timer.HasElapsed(units::second_t(4))){

    index.Set(0);
    tankDrive.TankDrive(1, 1);    //4 seconds after start stop index then run robot forward(full speed) for 1 second

  }else if(timer.HasElapsed(units::second_t(5))){

    tankDrive.TankDrive(0, 1);    //5 seconds after start turn robot left for half a second

  }else if(timer.HasElapsed(units::second_t(5.5))){

    intake.Set(1);
    tankDrive.TankDrive(1, 1);    //5.5 seconds after start turn on intake and run robot forward for 1 second

  }else if(timer.HasElapsed(units::second_t(6.5))){

    tankDrive.TankDrive(-1, -1);
    intake.Set(0);    //6.5 seconds after start run robot backwards for a second and turn off intake

  }else if(timer.HasElapsed(units::second_t(7.5))){

    tankDrive.TankDrive(0, -1);   //7.5 seconds after start turn robot back to starting direction for half a second

  }else if(timer.HasElapsed(units::second_t(8))){

    tankDrive.TankDrive(-1, -1);    //8 seconds after start run robot backward(full speed) for 1 second

  }else if(timer.HasElapsed(units::second_t(9))){

    tankDrive.TankDrive(0, 0);
    index.Set(1);   //9 seconds after start stop robot and run index

  }else if(timer.HasElapsed(units::second_t(11))){

    index.Set(0);   //11 seconds after start stop index

  }
  
  shooter.Set(0);

  //go forward then left a little bit while running intake and run again
  //during, run index 
  //turn right same amount as left turn then drive backwards up against hub
  //2 seconds later run index to shoot ball

  }


void TeleopPeriodic() override {  //this runs periodically throughout teleop

    // Drive with tank style using drivePad
    tankDrive.TankDrive(drivePad1.GetRawAxis(1), drivePad2.GetRawAxis(1));  //axis 1 and 3 from drivePad are to gauge the drive

    //attach mechanisms to mechpad
    index.Set(mechPad.GetRawButton(1));
    shooterFunction();
    climberFunction();
    intakeFunction();
  }

void shooterFunction(){

  if(mechPad.GetRawButton(8)){

      shooter.Set(shooterSpeed); //intake

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


