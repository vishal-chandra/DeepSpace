/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap; 

public class setArm extends Command {
  double position; 
  public setArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Robot.arm.setSlot(1);
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute(){ 
    //Robot.arm.tune();
    // if(Math.abs(OI.controller.getRawAxis(3)) > 0.1){
    //   Robot.arm.setPower(-OI.controller.getRawAxis(3)); 
    // }
    if(Robot.arm.getBall()) Robot.arm.fly.set(-0.2);
    else Robot.arm.fly.set(0.0); 

    this.position = Robot.arm.position;  

    Robot.arm.setPosition(this.position); 
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
    //end(); 
  }
}
