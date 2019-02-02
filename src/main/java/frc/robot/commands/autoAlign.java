package frc.robot.commands;
import frc.robot.Robot;
import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class autoAlign extends Command implements PIDOutput{
	private PIDController controller; 
	private double endTime; 
	private double angle; 
	private double speed; 
	
	private double pidOut; 
	
	private long onTargetMillis; 
	private long startTimeMillis; 
	
	
    public autoAlign() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveTrain); 
    	angle = -90.0; 
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	double kP = RobotMap.kP;
    	double kI = RobotMap.kI; 
    	double kD = RobotMap.kD; 
    	
    	Robot.driveTrain.resetGyro();
    	
    	controller = new PIDController(kP, kI, kD, Robot.driveTrain.gyroSensor, this);
    	controller.setInputRange(-180.0, 180.0);
    	controller.setOutputRange(-RobotMap.SPEED, RobotMap.SPEED); 
    	controller.setAbsoluteTolerance(RobotMap.TOLERANCE); 
    	controller.setContinuous();
    	controller.setSetpoint(angle); 
    	controller.enable(); 
    	startTimeMillis = System.currentTimeMillis(); 
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.driveTrain.driveCertainAmounts(pidOut, -pidOut); 
    	SmartDashboard.putBoolean("onTarget:", controller.onTarget()); 
    	SmartDashboard.putNumber("angle:", Robot.driveTrain.getYaw()); 
    	SmartDashboard.putData("pid:", controller);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if(!controller.onTarget()){
        	onTargetMillis = 0; 
        	return false; 
        }
        
        if(onTargetMillis == 0){
        	onTargetMillis = System.currentTimeMillis(); 
        }
        
        return System.currentTimeMillis() > onTargetMillis + 100 || controller.onTarget(); 
//        return controller.onTarget(); 
    }

    // Called once after isFinished returns true
    protected void end() {
    	controller.disable(); 
    	Robot.driveTrain.stop(); 
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
    
    public void pidWrite(double output){
    	synchronized(this){
    		pidOut = output; 
    		SmartDashboard.putNumber("pidOut:", pidOut); 
    		
    		
    	}
    }
}
