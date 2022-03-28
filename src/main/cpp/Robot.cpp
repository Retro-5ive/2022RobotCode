// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/GenericHID.h>
#include <frc/TimedRobot.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/Joystick.h> 
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/Timer.h>
#include <cameraserver/CameraServer.h>


class Robot : public frc::TimedRobot {      

double shooterSpeed = 0.7;
double indexSpeed = 0.5;
double intakeSpeed = 0.9;
bool toggle{false};

//Talon drive motors initialization and groupings
static const int leftFID = 1, leftBID = 3, rightFID = 2, rightBID = 4;
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_leftF{leftFID};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_leftB{leftBID};
frc::MotorControllerGroup left{m_leftF, m_leftB};

ctre::phoenix::motorcontrol::can::WPI_TalonFX m_rightF{rightFID};
ctre::phoenix::motorcontrol::can::WPI_TalonFX m_rightB{rightBID};
frc::MotorControllerGroup right{m_rightF, m_rightB};


frc::DifferentialDrive tankDrive{left, right};    //make left side and right side into one drive - tank drive


//Talon mechanism motors initialization
ctre::phoenix::motorcontrol::can::WPI_TalonFX shooter{7};
ctre::phoenix::motorcontrol::can::WPI_TalonFX climber{9};
ctre::phoenix::motorcontrol::can::WPI_TalonFX climber2{6};
ctre::phoenix::motorcontrol::can::WPI_TalonFX index{8};
ctre::phoenix::motorcontrol::can::WPI_TalonFX intake{5};

//Joysticks instantiation
frc::Joystick leftStick{0};    //port 0 is a joystick for drive
frc::Joystick rightStick{1};     //port 1 is joystick for drive
frc::GenericHID mechPad{2};       //port 2 is a gamepad for mechanisms

//timer
 frc::Timer timer;
 int counter = 0;





public:

void RobotInit() override { //This runs on initialization of the robot during teleop
    right.SetInverted(true);
    timer.Reset();
    timer.Start();
    frc::CameraServer::StartAutomaticCapture();
  }



void TeleopPeriodic() override {  //this runs periodically throughout teleop

    // Drive with tank style using drivePad
    tankDrive.TankDrive(rightStick.GetRawAxis(1), leftStick.GetRawAxis(1));  //axis 1 and 1 from drivePad are to gauge the drive

    //attach mechanisms to mechpad
    //both in and out so button 1 for in button 3 for out
    shooterFunction();
    indexFunction();
    intakeFunction();
    climberFunction();
    climber2Function();
    
  }


void shooterFunction(){ //velocity control for talonfx
  
  
if (mechPad.GetRawButtonPressed(3)){

  if(toggle){

    shooter.Set(0);
    toggle = false;

  }else {

    shooter.Set(shooterSpeed);
    toggle = true;

  }
}

  
}

void indexFunction(){

  if(mechPad.GetRawButton(8)){

      index.Set(indexSpeed);

  }else if (mechPad.GetRawButton(6)){

    index.Set(-(indexSpeed));
      
  }else{

    index.Set(0);

  }
}



void intakeFunction(){

  if(mechPad.GetRawButton(7)) {  //top triggers on mechPad

    intake.Set(intakeSpeed);

  }else if(mechPad.GetRawButton(5)){

    intake.Set(-(intakeSpeed));

  }else{ 

    intake.Set(0);

  }
}

void climberFunction(){ 

    if(rightStick.GetRawButton(1)){ //rightstick button 1
 
    climber.Set(-1);

  }else if(leftStick.GetRawButton(1)){  //leftstick button 1

    climber.Set(1);

  }else{

    climber.Set(0);
  }
}

void climber2Function(){

    if(rightStick.GetRawButton(4)){    //rightstick button 2

    climber2.Set(0.30);

  }else if(rightStick.GetRawButton(3)){    //leftstick button 2

    climber2.Set(-0.30);

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
