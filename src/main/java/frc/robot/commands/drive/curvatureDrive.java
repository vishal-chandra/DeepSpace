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
	double drive_straight_kp = -0.01;
	double min_command = 0.25; 
	double min_speed = -0.1; 
	double SKIM_GAIN = 0.5; 

	boolean driving_straight = false; 
	//double min_speed = 0.2;
	double left_command; 
	double right_command; 
	double angle_setPoint; 

	double leftUltra; 
	double rightUltra; 
	double ultra_kP = 0.05; 

	RampComponent left; 
	RampComponent right;
	  
    public curvatureDrive() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain); 
		requires(Robot.vision); 
		left = new RampComponent(4.0); 
    	right = new RampComponent(4.0);
	}

	double skim(double v){
		if(v > 1.0){
		  return -((v - 1.0) * this.SKIM_GAIN); 
		}
		else if(v < -1.0){
		  return -((v + 1.0) * this.SKIM_GAIN);
		}
		return 0;
	}

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
		//double rightTrigger = Robot.oi.xbox.getTriggerAxis(Hand.kRight);
		//SmartDashboard.putNumber("Right trigger:", rightTrigger); 
		Robot.vision.getValues();
		double forward = -Robot.oi.xbox.getY(GenericHID.Hand.kLeft);
		double turn = Robot.oi.xbox.getX(GenericHID.Hand.kRight); 

		// forward = Math.abs(forward) < 0.1 ? 0 : forward; 
		// turn = Math.abs(turn) < 0.1 ? 0 : turn;
		
		left_command  = forward + turn; 
		right_command = forward - turn; 

    // ramp the inputs to the desired setpoint 
    	// left_command = left.applyAsDouble(left_command); 
    	// right_command = right.applyAsDouble(right_command);

		// Robot.driveTrain.driveCertainAmounts(Math.abs(left_command) * left_command * 0.8 , 
		// 									 Math.abs(right_command) * right_command * 0.8);
		// // if(OI.controller.getRawButton(9)){
		// 	Robot.vision.changePipeline(0);
		// }
		// else if(OI.controller.getRawButton(10)){
		// 	Robot.vision.changePipeline(1);
		// }
		// System.out.println(driving_straight);
		// if(driving_straight){
		// 	double angle_error = angle_setPoint - Robot.driveTrain.getYaw(); 
		// 	left_command = -Robot.oi.xbox.getY(GenericHID.Hand.kLeft); 
		// 	right_command = -Robot.oi.xbox.getY(GenericHID.Hand.kLeft); 

		// 	double steering_adjust = 0.0; 
			
		// 	steering_adjust = drive_straight_kp * angle_error; 
			
		// 	left_command -= steering_adjust; 
		// 	right_command += steering_adjust; 
			
		// 	Robot.driveTrain.driveCertainAmounts(left_command, right_command);
		// }

		
		//Press B to follow target
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
			//Robot.driveTrain.driveCertainAmounts(left_command, right_command);
			Robot.driveTrain.setSpeed(left_command, right_command, true);	
		}
		
		if(Robot.driveTrain.drivingStraight){

			//System.out.println(driving_straight);
			if(driving_straight){
				double angle_error = angle_setPoint - Robot.driveTrain.getYaw(); 
				left_command = -Robot.oi.xbox.getY(GenericHID.Hand.kLeft); 
				right_command = -Robot.oi.xbox.getY(GenericHID.Hand.kLeft); 

				double steering_adjust = 0.0; 
			
				steering_adjust = drive_straight_kp * angle_error; 
			
				left_command -= steering_adjust; 
				right_command += steering_adjust; 
			
				//Robot.driveTrain.driveCertainAmounts(left_command, right_command);
				Robot.driveTrain.setSpeed(left_command, right_command, true);
			}
			if((Math.abs(forward) > 0.2 && Math.abs(turn) < 0.2) && !driving_straight) {
				angle_setPoint = Robot.driveTrain.getYaw(); 
				driving_straight = true;
			}
			else if((Math.abs(forward) < 0.2 && Math.abs(turn) > 0.2) ||
					(Math.abs(forward) > 0.2 && Math.abs(turn) > 0.2) || 
					(Math.abs(forward) < 0.2 && Math.abs(turn) < 0.2) ) {
					driving_straight = false; 
					Robot.driveTrain.curavtureDrive(forward, turn);
			}
			
		}
		else{

			Robot.driveTrain.curavtureDrive(forward, turn);
		}
		// if(Math.abs(Robot.oi.xbox.getY(GenericHID.Hand.kLeft)) < 0.2 && Math.abs(Robot.oi.xbox.getX(GenericHID.Hand.kRight)) > 0.2){
		// 	//driving_straight = false; 
		// 	Robot.driveTrain.curavtureDrive(-Robot.oi.xbox.getY(GenericHID.Hand.kLeft), Robot.oi.xbox.getX(GenericHID.Hand.kRight));
			
		// }
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
