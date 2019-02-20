/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class elevatorLIDAR extends Command {
  double position; 
  double current_position; 
  boolean finished;
  double tolerance = 5; 
  
  public elevatorLIDAR(double position) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator); 
    this.position = position; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    finished = false;
    current_position = Robot.elevator.lidar.getDistance(); 
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if(current_position < position && !(Robot.elevator.carriage_up.get() && Robot.elevator.stage2_up.get())){
      Robot.elevator.raiseElevator();
    }
    else if(current_position > position && !Robot.elevator.elevator_down.get()){
      Robot.elevator.lowerElevator();
    }
    else if(Math.abs(position - current_position) < tolerance) {
      finished = true; 
      Robot.elevator.stopElevator();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.stopElevator();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.stopElevator();
  }
}
