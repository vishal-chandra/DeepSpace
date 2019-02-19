package frc.robot.commands.vision;
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
    
    double start_time = 0.0; 
    boolean done; 

    public followTarget() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.driveTrain); 
        requires(Robot.vision); 
        start_time = System.currentTimeMillis(); 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        done = false; 
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Robot.vision.getValues();

    	double heading_error =  -(Robot.vision.x); 
	    double steering_adjust = 0.0; 
	    if(Robot.vision.x > 1.0){
	    	steering_adjust = Kp * heading_error + min_command; 
	    }
	    else if(Robot.vision.x < 1.0){
	    	steering_adjust = Kp*heading_error - min_command; 
	    }
	    	
	    left_command += steering_adjust; 
	    right_command -= steering_adjust; 
	    // have to invert left and right cause weird stuff 
	    Robot.driveTrain.driveCertainAmounts(left_command, right_command);
			    
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(System.currentTimeMillis() - start_time < 1500){
            return true; 
        }
        return done;
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
