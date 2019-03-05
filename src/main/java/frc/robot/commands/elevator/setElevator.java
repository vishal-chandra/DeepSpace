/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class setElevator extends Command {
  double position; 
  public setElevator() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Robot.elevator.setPID(RobotMap.elevator_position_kF, RobotMap.elevator_position_kP, RobotMap.elevator_position_kI, RobotMap.elevator_position_kD);
    // Motion Magic: 0 
    // only for tuning purposes
    Robot.elevator.setSlot(0); 

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    this.position = Robot.elevator.position; 
    Robot.elevator.setPosition(this.position); 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
