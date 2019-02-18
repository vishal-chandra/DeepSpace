package frc.robot.commands;
import frc.robot.OI;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class curvatureDrive extends Command {
	double Kp = -0.040;
	double drive_straight_kp = -0.02;
	double min_command = 0.08; 
	double min_speed = 0.1;
	double left_command; 
	double right_command; 
	double angle_setPoint; 
    public curvatureDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain); 
    	requires(Robot.vision); 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		//double rightTrigger = Robot.oi.xbox.getTriggerAxis(Hand.kRight);
		//SmartDashboard.putNumber("Right trigger:", rightTrigger); 
		Robot.vision.getValues();
		angle_setPoint = Robot.driveTrain.getYaw(); 
		left_command = Robot.oi.xbox.getX(GenericHID.Hand.kRight) + Robot.oi.xbox.getY(GenericHID.Hand.kLeft) ; 
		
		right_command = Robot.oi.xbox.getX(GenericHID.Hand.kRight) - Robot.oi.xbox.getY(GenericHID.Hand.kLeft) ; 
		if(OI.controller.getRawButton(9)){
			Robot.vision.changePipeline(0);

		}
		else if(OI.controller.getRawButton(10)){
			Robot.vision.changePipeline(1);

		}
		else if(OI.controller.getRawButton(6)){
	    	SmartDashboard.putNumber("Follow: ", Robot.vision.x); 

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
		else if(OI.xbox.getRawButton(1)){
			
			// Robot.vision.getPhotoSensorValues();
      		// String value = Robot.vision.sensorValues;

     		//  double angle = 0;
    		//  double speed = 0.25;

      		// if(value.equals("100")) angle = -0.15; //slight left
      		// else if(value.equals("110")) angle = -0.07; //very slight left

      		// else if(value.equals("001")) angle = 0.15; //slight right
      		// else if(value.equals("011")) angle = 0.7; //very slight right

     		// else if(value.equals("010")) angle = 0; //go straight
      		// else if(value.equals("101") || value.equals("111") || value.equals("000")) 
     		// {
          	// 	angle = 0; 
          	// 	speed = 0; //stop, there's been an error
     		// }

			//  Robot.driveTrain.curavtureDrive(speed, angle);
		}
//		else if(OI.controller.getRawButton(8)){
//			left_command = 0.5; 
//			right_command = 0.5; 
//			double angle_error = -Robot.driveTrain.getYaw(); 
//			double steering_adjust = 0.0; 
//			
//			steering_adjust = drive_straight_kp * angle_error; 
//			left_command -= steering_adjust; 
//			right_command += steering_adjust; 
//			
//			Robot.driveTrain.driveCertainAmounts(left_command, right_command);
//		}
//		else if(Math.abs(Robot.oi.xbox.getY(GenericHID.Hand.kLeft)) > 0 &&Math.abs(Robot.oi.xbox.getX(GenericHID.Hand.kRight)) < 0.1 ){
//			double angle_error = angle_setPoint - Robot.driveTrain.getYaw(); 
//			left_command = -Robot.oi.xbox.getY(GenericHID.Hand.kLeft); 
//			right_command = -Robot.oi.xbox.getY(GenericHID.Hand.kLeft); 
//
//			double steering_adjust = 0.0; 
//			
//			steering_adjust = drive_straight_kp * angle_error; 
//			if(Math.abs(Robot.oi.xbox.getY(GenericHID.Hand.kLeft)) > 0.3){
//				min_speed = -0.1;
//			} else{
//				min_speed = 0.1; 
//			}
//			left_command -= steering_adjust + min_speed; 
//			right_command += steering_adjust + min_speed; 
//			
//			Robot.driveTrain.driveCertainAmounts(left_command, right_command);
//		}
		else{
			Robot.driveTrain.curavtureDrive(-Robot.oi.xbox.getY(GenericHID.Hand.kLeft), Robot.oi.xbox.getX(GenericHID.Hand.kRight));
			
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
