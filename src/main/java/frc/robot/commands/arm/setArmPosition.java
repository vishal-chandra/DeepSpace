/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class setArmPosition extends Command {
  public double position; 
  public setArmPosition(double position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = position; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.arm.position = this.position; 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Robot.arm.position = this.position; 

    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.arm.position = this.position; 

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.arm.position = this.position; 

  }
}
