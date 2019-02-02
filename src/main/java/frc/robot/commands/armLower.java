package frc.robot.commands;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class armLower extends Command {

    public armLower() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	//requires(Robot.arm); 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	//Robot.arm.lowerArm(); 
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
//    	if(Robot.arm.armIsDown()){
//    		Robot.arm.armStop();
//    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false; 
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.arm.armStop(); 
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	//Robot.arm.armStop(); 
    }
}
