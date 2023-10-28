// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.RobotContainer;


public class driverobot extends Command {
  /** Creates a new driverobot. */
  double deadzone = 0.3;	//variable for amount of deadzone
    double y = 0;           //variable for forward/backward movement
    double x = 0;           //variable for side to side movement
    double turn = 0;        //variable for turning movement
XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  public driverobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_robotDrive);
    }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
x=0;
y=0;
turn=0;


if(m_driverController.getLeftX() > deadzone || m_driverController.getLeftX() < -deadzone) {
	y = m_driverController.getLeftX();
}

if(m_driverController.getLeftY() > deadzone || m_driverController.getLeftY() < -deadzone) {
	x = m_driverController.getLeftY();
}

if(m_driverController.getRightX() > deadzone || m_driverController.getRightX() < -deadzone){
	turn = m_driverController.getRightX();
}
/*if(m_driverController.getXButton()){
    y=0;
    x=0;
    turn=0;
    RobotContainer.m_robotDrive.setX();
}*/
  if(x==0 && y==0 && turn == 0) {
    RobotContainer.m_robotDrive.setX();
  } else {
  RobotContainer.m_robotDrive.drive(
                    
    y/5,//forwards(divided by 10)
    -x/5,//sideways(divided by 10)
    -turn/5,//rotation(divided by 10)
    RobotContainer.fieldoriented.getSelected(),//field oriented
    RobotContainer.ratelimitChooser.getSelected()//limit max speed
    );
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
