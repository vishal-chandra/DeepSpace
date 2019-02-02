package frc.robot.commands;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class followTarget extends Command {
	float Kp = -0.1f;
	float min_command = 0.05f; 
	double left_command = 0; 
	double right_command = 0; 

    public followTarget() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain); 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("Follow: ", Robot.vision.x); 
//    	double heading_error =  -(Robot.vision.x); 
//    	double steering_adjust = 0.0; 
//    	if(Robot.vision.x > 1.0){
//    		steering_adjust = Kp * heading_error - min_command; 
//    	}
//    	else if(Robot.vision.x < 1.0){
//    		steering_adjust = Kp*heading_error + min_command; 
//    	}
//    	
//    	left_command += steering_adjust; 
//    	right_command -= steering_adjust; 
//    	
//    	Robot.driveTrain.driveCertainAmounts(left_command, right_command);
    	
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
