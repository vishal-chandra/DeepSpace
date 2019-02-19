/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.GenericHID;


public class alignToWall extends Command {
  double leftUltra; 
  double rightUltra; 
  double left_command; 
	double right_command; 

  double kP = 0.05; 
  
  public alignToWall() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.vision);
    requires(Robot.driveTrain); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // Robot.vision.leftUltra();
    // Robot.vision.rightUltra();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    leftUltra = Robot.vision.leftUltra();
    rightUltra = Robot.vision.rightUltra();

    double error = rightUltra - leftUltra; 

    left_command = Robot.oi.xbox.getX(GenericHID.Hand.kRight) + Robot.oi.xbox.getY(GenericHID.Hand.kLeft) ; 
		
		right_command = Robot.oi.xbox.getX(GenericHID.Hand.kRight) - Robot.oi.xbox.getY(GenericHID.Hand.kLeft) ; 

    double steering_adjust = error * kP; 

    left_command -= steering_adjust; 
    right_command += steering_adjust; 

    Robot.driveTrain.driveCertainAmounts(left_command, right_command); 

    

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.stop(); 
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.driveTrain.stop(); 
  }
}
