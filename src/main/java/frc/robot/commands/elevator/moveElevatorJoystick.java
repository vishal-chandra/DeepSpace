/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class moveElevatorJoystick extends Command {
  public moveElevatorJoystick() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.elevator);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if(Robot.elevator.elevator_down.get()){
    //   Robot.elevator.resetEncoder();
    // }
    double power = -Robot.oi.xbox.getY(GenericHID.Hand.kRight); 
    SmartDashboard.putNumber("Power applied to elevator", power);
    
    //Robot.elevator.setPower(power); //arb ff
    if(power > 0 && !(Robot.elevator.carriage_up.get() && Robot.elevator.stage2_up.get())) Robot.elevator.setPower(power);
    else Robot.elevator.setPower(power); 
    //else if(power < 0 && !(Robot.elevator.elevator_down.get())) Robot.elevator.setPower(power); 

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.elevator.position = Robot.elevator.getPosition();
    Robot.elevator.stopElevator();


  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.elevator.position = Robot.elevator.getPosition();

    Robot.elevator.stopElevator();
  }
}
