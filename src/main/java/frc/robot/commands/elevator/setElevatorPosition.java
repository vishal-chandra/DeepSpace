/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class setElevatorPosition extends Command {
  public double position; 
  public setElevatorPosition(double position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.position = position; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.elevator.position = position; 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.elevator.position = position; 

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    Robot.elevator.position = position; 

    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.position = position; 

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.position = position; 

  }
}
