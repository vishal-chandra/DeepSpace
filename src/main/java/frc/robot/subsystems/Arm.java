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
	
	SpeedController body1;
	SpeedController body2; 
//	WPI_TalonSRX arm; 
	TalonSRX arm; 
 
	Encoder bodyEncoder;
	
	DigitalInput bodyUp; 
	DigitalInput bodyDown; 
	
	DigitalInput armUp; 
	DigitalInput armDown; 

	public Arm(){
//		arm = new WPI_TalonSRX(RobotMap.ARM_MOTOR); 
		arm = new TalonSRX(RobotMap.ARM_MOTOR) ;
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
    
    public void raiseBody(){
    	body1.set(0.8);
    	body2.set(0.8);

    }
    
    public void lowerBody(){
    	body1.set(-0.8);
    	body2.set(-0.8);

    }
    public void bodyStop(){
    	body1.set(0.0);
    	body2.set(0.0);

    }
    public void moveArm(double distance){
    	arm.set(ControlMode.Position, distance);
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
    // BODY
    public boolean bodyIsUp(){
    	return bodyUp.get(); 
    }
    
    public boolean bodyIsDown(){
    	return bodyDown.get(); 
    }
    // ARM
    public boolean armIsUp(){
    	return bodyUp.get(); 
    }
    
    public boolean armIsDown(){
    	return bodyDown.get(); 
    }
    //ENCODERS
    public double getBodyEncoder(){
    	return bodyEncoder.getDistance();
    }
    
    public double getArmEncoder(){
    	return arm.getSensorCollection().getQuadraturePosition();
    }
    
    public void resetEncoder(){
    	bodyEncoder.reset();
    }
    public void updateSmartDashboard(){
//    	SmartDashboard.putBoolean("Body Up:", bodyIsUp()); 
//    	SmartDashboard.putBoolean("Body Down:", bodyIsDown()); 
//    	
//    	SmartDashboard.putBoolean("Arm Up:", armIsUp()); 
//    	SmartDashboard.putBoolean("Arm Down:", armIsDown()); 
//    	
//    	SmartDashboard.putNumber("Body Encoder:", getBodyEncoder()); 
    	SmartDashboard.putNumber("Arm Encoder:", getArmEncoder()); 

    	
    	
    }
    
    
}

