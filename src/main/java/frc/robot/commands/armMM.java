/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class armMM extends Command {

  double currentPosition; 
  int desiredPosition;
  public armMM(int setpoint) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm);
    this.desiredPosition = setpoint;  
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.setSlot(0); 
    Robot.arm.move_MM(desiredPosition);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.arm.onTarget_MM(desiredPosition);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.arm.armStop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.arm.armStop(); 
  }
}
