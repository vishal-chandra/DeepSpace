package frc.robot.commands.drive;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class turnRight90 extends Command {
	double angle; 
	boolean stop = false; 
    public turnRight90() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.driveTrain.rotateClockWise();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	angle = Robot.driveTrain.getYaw();
    	if(angle > 90.0){
    		Robot.driveTrain.stop(); 
    		stop = true; 
    	}
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
		Robot.driveTrain.stop(); 

    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
		Robot.driveTrain.stop(); 

    }
}
