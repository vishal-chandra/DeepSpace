package frc.robot.commands.drive;
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
	double Kp = 0.04;
	//double drive_straight_kp = -0.02;
	double min_command = 0.25; 
	//double min_speed = 0.2;
	double left_command; 
	double right_command; 
	double angle_setPoint; 

	double leftUltra; 
	double rightUltra; 
	double ultra_kP = 0.05; 
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
		// if(OI.controller.getRawButton(9)){
		// 	Robot.vision.changePipeline(0);

		// }
		// else if(OI.controller.getRawButton(10)){
		// 	Robot.vision.changePipeline(1);

		// }
		if(OI.xbox.getRawButton(2)){
	    	SmartDashboard.putNumber("Follow: ", Robot.vision.x); 

			double heading_error = (Robot.vision.x); 
	    	double steering_adjust = 0.0; 
	    	if(Robot.vision.x > 1.0){
	    		steering_adjust = Kp * heading_error + min_command; 
	    	}
	    	else if(Robot.vision.x < -1.0){
	    		steering_adjust = Kp*heading_error - min_command; 
			}
			//System.out.println("====="); 
			//System.out.println("Robotx:" + Robot.vision.x); 
			//System.out.println("steering adjust" + steering_adjust); 
	    	left_command += steering_adjust; 
			right_command -= steering_adjust; 

			//System.out.println("Left command: " + left_command + "Right Command: " + right_command); 
	    	// have to invert left and right cause weird stuff 
	    	Robot.driveTrain.driveCertainAmounts(left_command, right_command);
			
		}
		else if(OI.xbox.getRawButton(1)){
			
			// leftUltra = Robot.vision.leftUltra();
    		// rightUltra = Robot.vision.rightUltra();

    		// double error = rightUltra - leftUltra; 

    		// left_command = Robot.oi.xbox.getX(GenericHID.Hand.kRight) + Robot.oi.xbox.getY(GenericHID.Hand.kLeft) ; 
		
			// right_command = Robot.oi.xbox.getX(GenericHID.Hand.kRight) - Robot.oi.xbox.getY(GenericHID.Hand.kLeft) ; 

    		// double steering_adjust = error * ultra_kP; 

    		// left_command -= steering_adjust; 
    		// right_command += steering_adjust; 

    		// Robot.driveTrain.driveCertainAmounts(left_command, right_command); 
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
