package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.drive.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;



public class DriveTrain extends Subsystem {
	//private SpeedController bottomLeft; 
	//private SpeedController bottomRight;

	private WPI_VictorSPX frontLeftSpx; 
	private WPI_VictorSPX frontRightSpx; 
	private WPI_TalonSRX backLeftSrx; 
	private WPI_TalonSRX backRightSrx;

	public double MAXIMUM_VELOCITY = 3600; // take 90-80% of this 


	double left_arbfeedfwd = 0.0; 
	double right_arbfeedfwd = 0.0; 
	// left talon id: 1
	// right talon id: 0 
	SpeedControllerGroup left; 
	SpeedControllerGroup right; 
	 
	public Encoder leftEncoder; 
	public Encoder rightEncoder;
	
	public DigitalInput colorSensor; 


	public boolean backwards = false; 
	public boolean quickTurn = false; 
	public AHRS gyroSensor; 

//	RobotDrive robotDrive; 
	DifferentialDrive robotDrive; 

	public DriveTrain(){
//		leftEncoder = new Encoder(RobotMap.LEFT_ENCODER_ONE, RobotMap.LEFT_ENCODER_TWO, false, Encoder.EncodingType.k4X);
//		rightEncoder = new Encoder(RobotMap.RIGHT_ENCODER_ONE, RobotMap.RIGHT_ENCODER_TWO, false, Encoder.EncodingType.k4X);
		backLeftSrx = new WPI_TalonSRX(RobotMap.BACK_LEFT_MOTOR);//1 

		frontLeftSpx = new WPI_VictorSPX(RobotMap.FRONT_LEFT_MOTOR); //2 
		backLeftSrx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		left = new SpeedControllerGroup(backLeftSrx, frontLeftSpx); 

		frontRightSpx = new WPI_VictorSPX(RobotMap.FRONT_RIGHT_MOTOR); // 3 
		backRightSrx = new WPI_TalonSRX(RobotMap.BACK_RIGHT_MOTOR); // 4

		 
		right = new SpeedControllerGroup(backRightSrx, frontRightSpx); 
		backRightSrx.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

		frontRightSpx.follow(backRightSrx); 
		frontLeftSpx.follow(backLeftSrx);
		
		backLeftSrx.getSensorCollection().setQuadraturePosition(0, 10); 
    	backRightSrx.getSensorCollection().setQuadraturePosition(0, 10);



		robotDrive = new DifferentialDrive(left, right); 
//		robotDrive = new RobotDrive(topLeft, bottomLeft,  topRight, bottomRight); 
		robotDrive.setSafetyEnabled(false);
		
		gyroSensor = new AHRS(SPI.Port.kMXP);

		gyroSensor.reset(); 
		

		


	}

	
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		// setDefaultCommand(new MySpecialCommand());
		//setDefaultCommand(new drive()); 
		setDefaultCommand(new curvatureDrive()); 

	}
	
	public void rotateClockWise(){
		robotDrive.tankDrive(0.5, -0.5);
	}
	
	public void rotateCounterClockWise(){
		robotDrive.tankDrive(-0.5, 0.5); 
	}

	public void setSpeed(double left_input, double right_input){
		
		backLeftSrx.set(ControlMode.Velocity, left_input * MAXIMUM_VELOCITY, DemandType.ArbitraryFeedForward, left_arbfeedfwd); 
		backRightSrx.set(ControlMode.Velocity, right_input * MAXIMUM_VELOCITY, DemandType.ArbitraryFeedForward, right_arbfeedfwd); 

	}
	public void driveNow(Joystick left, Joystick right){
		
		frontLeftSpx.setInverted(backwards);
		backLeftSrx.setInverted(backwards);
		frontRightSpx.setInverted(backwards);
		backRightSrx.setInverted(backwards);
		if(backwards) robotDrive.tankDrive(-right.getY(), -left.getY(), true); 
		else robotDrive.tankDrive(-left.getY(),  -right.getY(), true);

		

	}
	
	public void curavtureDrive(double forward, double turn){
		frontLeftSpx.setInverted(backwards);
		backLeftSrx.setInverted(backwards);
		frontRightSpx.setInverted(backwards);
		backRightSrx.setInverted(backwards);
		if(forward < -0.1 ) turn = -turn; 
//		if(Math.abs(forward) < 0.05 && Math.abs(turn) > 0){
//			robotDrive.tankDrive(-turn, turn);
//		}
//		else{
//			robotDrive.curvatureDrive(forward, turn, false);
//		}
		forward = Math.abs(forward) < 0.2 ? 0 : forward; 
		turn = Math.abs(turn) < 0.2 ? 0 : turn; 
		robotDrive.curvatureDrive(forward * Math.abs(forward), turn * Math.abs(turn), true);

	}

	public void driveCertainAmounts(double left, double right){
		robotDrive.tankDrive(left, right); 
	}

	public void stop(){
		robotDrive.tankDrive(0.0,0.0);
	}

	public void driveSlow(){
		robotDrive.tankDrive(0.5, 0.5); 
	}
	public void calibrateGyro(){
		gyroSensor.reset();
	}
	public double getYaw(){
		return gyroSensor.getYaw(); 
	}

	public void resetGyro(){
		gyroSensor.reset(); 
	}
	
	public void resetEncoders(){
    	backLeftSrx.getSensorCollection().setQuadraturePosition(0, 10); 
    	backRightSrx.getSensorCollection().setQuadraturePosition(0, 10);

	}
	public double getLeft(){
		return backLeftSrx.getSensorCollection().getQuadraturePosition();
	}
	public double getRight(){
		return backRightSrx.getSensorCollection().getQuadraturePosition();
	}

	public void updateSmartDashboard(){
		//		SmartDashboard.putNumber("Ultrasonic sensor one", getDistanceOne()); 
		//		SmartDashboard.putNumber("Ultrasonic sensor two", getDistanceTwo()); 

		//SmartDashboard.putNumber("Drive Encoder One - Right Side", getEncoderOne());

		SmartDashboard.putNumber("Drivetrain Angle:", getYaw()); 
		SmartDashboard.putNumber("Left Encoder: ", getLeft());
		SmartDashboard.putNumber("Right Encoder: ", getRight()); 


	}
	//

}
