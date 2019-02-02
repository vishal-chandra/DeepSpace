package frc.robot.commands;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class driveStraight extends Command {
	double left_command = 0; 
	double right_command = 0; 
	
	double initial_right = 0;
	double initial_left = 0;
	
	double steering_adjust = 0.0; 
	double drive_straight_kp = -0.02;

	
	

    public driveStraight(double left_command, double right_command) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain); 
    	Robot.driveTrain.resetGyro();
    	
    	this.left_command = left_command;
    	this.right_command = right_command;
    	initial_left = left_command;
    	initial_right = right_command; 

    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	
    	Robot.driveTrain.resetGyro();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	left_command = initial_left; 
    	right_command = initial_right;
    	double angle_error = -Robot.driveTrain.getYaw(); 
    	steering_adjust = drive_straight_kp * angle_error; 
    	left_command -= steering_adjust; 
    	right_command += steering_adjust; 
    	
    	Robot.driveTrain.driveCertainAmounts(left_command, right_command);
    	
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
