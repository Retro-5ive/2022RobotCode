// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
#include <frc/GenericHID.h>
#include <frc/TimedRobot.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/Joystick.h> 
#include "rev/SparkMaxPIDController.h"
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Timer.h>

#include <rev/CANSparkMax.h>


class Robot : public frc::TimedRobot {      

double shooterSpeed = 0.25;

//Talon drive motors initialization and groupings
static const int leftFID = 9, leftBID = 10, rightFID = 8, rightBID = 14;
rev::CANSparkMax  m_leftF{leftFID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax  m_leftB{leftBID, rev::CANSparkMax::MotorType::kBrushless};
frc::MotorControllerGroup left{m_leftF, m_leftB};

rev::CANSparkMax  m_rightF{rightFID, rev::CANSparkMax::MotorType::kBrushless};
rev::CANSparkMax  m_rightB{rightBID, rev::CANSparkMax::MotorType::kBrushless};
frc::MotorControllerGroup right{m_rightF, m_rightB};


frc::DifferentialDrive tankDrive{left, right};    //make left side and right side into one drive - tank drive


//Talon mechanism motors initialization
ctre::phoenix::motorcontrol::can::WPI_TalonSRX intake{1};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX shooter{0};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX climber{2};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX climber2{7};
ctre::phoenix::motorcontrol::can::WPI_TalonSRX index{4};


//Joysticks instantiation
frc::Joystick leftStick{0};    //port 0 is a joystick for drive
frc::Joystick rightStick{1};     //port 1 is joystick for drive
frc::GenericHID mechPad{2};       //port 2 is a gamepad for mechanisms

//timer
 frc::Timer timer;


public:

void RobotInit() override { //This runs on initialization of the robot during teleop
    right.SetInverted(true);
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
    tankDrive.TankDrive(leftStick.GetRawAxis(1), rightStick.GetRawAxis(1));  //axis 1 and 1 from drivePad are to gauge the drive
    intake.Set(mechPad.GetRawAxis(1));

    //attach mechanisms to mechpad
    //both in and out so button 1 for in button 3 for out
    shooterFunction();
    climberFunction();
    climber2Function();
    indexFunction();
  }


void shooterFunction(){

  if(mechPad.GetRawButton(8)){

      shooter.Set(1);

  }else{

    shooter.Set(0);
      
  }
}

void indexFunction(){

  if(mechPad.GetRawButton(1)){

      index.Set(1);

  }else if (mechPad.GetRawButton(3)){

    index.Set(-1);
      
  }else{

    index.Set(0);

  }
}


void climberFunction(){ 

    if(rightStick.GetRawButton(1)){ //rightstick button 1
 
    climber.Set(1);

  }else if(leftStick.GetRawButton(1)){  //leftstick button 1

    climber.Set(-1);

  }else{

    climber.Set(0);
  }
}

void climber2Function(){

    if(rightStick.GetRawButton(2)){    //rightstick button 2

    climber2.Set(1);

  }else if(leftStick.GetRawButton(2)){    //leftstick button 2

    climber2.Set(-1);

  }else{

    climber2.Set(0);
  }
}


};






#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif


