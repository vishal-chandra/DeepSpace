/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.GenericHID;


public class curvaturePIDControl extends Command {
  double forward = 0; 
  double turn = 0; 
  double left_command = 0; 
  double right_command = 0; 
  double SKIM_GAIN = 0.5; 

  double lastValue; 
  long lastTime; 
  double maxChangePerMillis = 0.0; // TODO 
  public curvaturePIDControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    forward = -Robot.oi.xbox.getY(GenericHID.Hand.kLeft);
    turn = Robot.oi.xbox.getX(GenericHID.Hand.kRight);

    left_command = forward + turn; 
    right_command = forward - turn; 

    // adjusts the left and right commands to be more smooth and accurate 
    double adjusted_left = left_command + skim(right_command); 
    double adjusted_right = right_command + skim(left_command); 

  }
  // sets a ramp rate on the input 
  double ramp(double value){
    return 0.0; // TODO
  }

  double skim(double v){
    if(v > 1.0){
      return -((v - 1.0) * this.SKIM_GAIN); 
    }
    else if(v < -1.0){
      return -((v + 1.0) * this.SKIM_GAIN);
    }
    return 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    

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
