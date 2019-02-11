package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Arm extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	
    WPI_TalonSRX arm; 
    SpeedController fly; 
 
	
	
	
	DigitalInput armUp; 
    DigitalInput armDown; 
    public double position;

	public Arm(){
//		arm = new WPI_TalonSRX(RobotMap.ARM_MOTOR); 
        fly = new Talon(RobotMap.FLY);
        arm = new WPI_TalonSRX(RobotMap.ARM_MOTOR) ;
        arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        arm.configNominalOutputForward(0, 30);
        arm.configNominalOutputReverse(0, 30);
        arm.configPeakOutputForward(0.5, 30); 
        arm.configPeakOutputReverse(-0.5, 30);

        // POSITION: 0 
    this.setPID(RobotMap.ARM_POSITION_SLOT, RobotMap.arm_position_kF, 
      RobotMap.arm_position_kP, RobotMap.arm_position_kI, 
      RobotMap.arm_velocity_kD); 
    // VELOCITY: 1
    this.setPID(RobotMap.ARM_VELOCITY_SLOT, RobotMap.arm_velocity_kF,
      RobotMap.arm_velocity_kP, RobotMap.arm_velocity_kI,
      RobotMap.arm_velocity_kD);
//		body1 = new Talon(RobotMap.BODY_MOTOR_ONE); 
//		body2 = new Talon(RobotMap.BODY_MOTOR_TWO); 
//
//		bodyEncoder = new Encoder(RobotMap.BODY_ENCODER_PORT_ONE, RobotMap.BODY_ENCODER_PORT_TWO, false, Encoder.EncodingType.k4X);
//		
//		bodyUp = new DigitalInput(RobotMap.BODY_UP_SWITCH); 
//		bodyDown = new DigitalInput(RobotMap.BODY_DOWN_SWITCH); 
//		armUp = new DigitalInput(RobotMap.ARM_UP_SWITCH); 
//		armDown = new DigitalInput(RobotMap.ARM_DOWN_SWITCH); 

		
		
		arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    
    public void setPosition(){
        arm.selectProfileSlot(RobotMap.ARM_POSITION_SLOT, 0);
    	arm.set(ControlMode.Position, this.position);
    }

    public void setSpeed(double speed){
        arm.selectProfileSlot(RobotMap.ARM_VELOCITY_SLOT, 0);

        arm.set(ControlMode.Velocity, speed); 
      }

    public void raiseArm(){
    	arm.set(ControlMode.PercentOutput, 0.4);
//    	arm.set(0.1); 	
    }
    
    public void lowerArm(){
    	arm.set(ControlMode.PercentOutput, -0.4);
//    	arm.set(-0.1);
    }
    
    public void armStop(){
    	arm.set(ControlMode.PercentOutput, 0.0);
//    	arm.set(0.0);
    }
    public void setPID(int slot,double kF, double kP, double kI, double kD){
        arm.config_kF(slot, kF, 30); 
        arm.config_kP(slot, kP, 30); 
        arm.config_kI(slot, kI, 30); 
        arm.config_kD(slot, kD, 30); 
      }
   
    
    public double getArmEncoder(){
    	return arm.getSensorCollection().getQuadraturePosition();
    }
    
    public void resetArm(){
        arm.getSensorCollection().setQuadraturePosition(0, 30);

    }
    public void intake(){
        fly.set(0.5);
    }

    public void shoot(){
        fly.set(-0.5); 
    }

    public void setSlot(int slot){
        arm.selectProfileSlot(slot, 0);
      }
    public void updateSmartDashboard(){

    	SmartDashboard.putNumber("Arm Encoder:", getArmEncoder()); 

    	
    	
    }
    
    
}

