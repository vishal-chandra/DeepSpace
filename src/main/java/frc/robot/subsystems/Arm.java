package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.setArm;
import frc.robot.commands.moveElevatorJoystick;
import frc.robot.commands.armHold;
import frc.robot.commands.toggleArmHold;

/**
 *
 */
public class Arm extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	
    public WPI_TalonSRX arm; 
    SpeedController fly; 
    Preferences preferences; 
 
	public DigitalInput armUp; 
    public DigitalInput armDown; 
    double MAX_ARM_POSITION = -1700; 
    public double position;
    public DigitalInput ball_intake;
    public DigitalInput hatch_pickup;

    public boolean hasHatch;
    public boolean hold; 

    public double HATCH_LOW_POWER = 0.2; 
    public double HATCH_HIGH_POWER = 0.1; 
    public double NO_HATCH_LOW_POWER = 0.1; 

    public double BALL_HIGH_POWER = 0.1; 

    public double mm_kP = 0.0; 
    public double mm_kI = 0.0; 
    public double mm_kD = 0.0; 
    public double mm_kF = 0.0; 
    public int kTimeoutMs = 5; 

    int CRUISE_VELOCITY  = 251; // TODO 
    int ACCELERATION = 1004; // TODO

    public static  double arm_position_kF = 0.0; 
	public static  double arm_position_kP = 0.001; 
	public static  double arm_position_kI = 0.0; 
	public static  double arm_position_kD = 0.0;

	public Arm(double position){
        hold = false;
        preferences = Preferences.getInstance();
        preferences.putDouble("Arm Position kP", 0.001); 
        preferences.putDouble("Arm Position kI", 0.0); 

        preferences.putDouble("Arm Position kD", 0.0); 
        preferences.putDouble("Arm position setPoint:", position); 



//		arm = new WPI_TalonSRX(RobotMap.ARM_MOTOR); 
        this.position = position; 
        fly = new Talon(RobotMap.FLY);
        arm = new WPI_TalonSRX(RobotMap.ARM_ACTUATOR) ;

        ball_intake = new DigitalInput(RobotMap.BALL_INTAKE_SWITCH);
        hatch_pickup = new DigitalInput(RobotMap.HATCH_PICKUP_SWITCH);
        //armUp = new DigitalInput(RobotMap.ARM_UP_SWITCH); 
        //armDown = new DigitalInput(RobotMap.ARM_DOWN_SWITCH); 
        
        arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
        arm.configNominalOutputForward(0, 10);
        arm.configNominalOutputReverse(0, 10);
        arm.configPeakOutputForward(1.0, 10); 
        arm.configPeakOutputReverse(-1.0, 10);
        arm.getSensorCollection().setQuadraturePosition(0, 10);  


        arm.setSensorPhase(true); 
        arm.setInverted(true); 


        // POSITION: 0 
        this.setPID(0, arm_position_kF, 
            arm_position_kP, arm_position_kI, arm_position_kD); 
    
    // Motion Magic: 1
        arm.configMotionCruiseVelocity(CRUISE_VELOCITY, kTimeoutMs); 
        arm.configMotionAcceleration(ACCELERATION, kTimeoutMs); 
        this.setPID(1, mm_kF, mm_kP, mm_kI, mm_kD); 
    // this.setPID(RobotMap.ARM_VELOCITY_SLOT, RobotMap.arm_velocity_kF,
    //   RobotMap.arm_velocity_kP, RobotMap.arm_velocity_kI,
    //   RobotMap.arm_velocity_kD);

		
		
		arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
	}
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        //setDefaultCommand(new setArm()); 
        //setDefaultCommand(new moveElevatorJoystick());
        setDefaultCommand(new armHold());
    }
    
    public void move_MM(double targetPos){
        arm.configAllowableClosedloopError(0, 10, kTimeoutMs); 
        arm.set(ControlMode.MotionMagic, targetPos); 
    }

    public boolean onTarget_MM(double targetPos){
        double tolerance = 10; 
        double currentPos = arm.getSelectedSensorPosition(); 
        return Math.abs(targetPos - currentPos) < tolerance; 
    }
    
    public void setPosition(double setpoint){
        //arm.selectProfileSlot(RobotMap.ARM_POSITION_SLOT, 0);
        arm.set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, Math.cos(this.getArmEncoder() / 4096.0 * 360) * 0.165); 

        //arm.set(ControlMode.Position, this.position);
    }
    

    public void setSpeed(double speed){
        //arm.selectProfileSlot(RobotMap.ARM_VELOCITY_SLOT, 0);
        arm.set(ControlMode.Velocity, speed); 
    }

    public void raiseArm(){
        arm.set(ControlMode.PercentOutput, 0.1);	
    }
    
    public void lowerArm(){
    	arm.set(ControlMode.PercentOutput, -0.2);
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

    public double getVelocity(){
        return arm.getSensorCollection().getQuadratureVelocity();
    }
    
    public void resetArm(){
        arm.getSensorCollection().setQuadraturePosition(0, 30);

    }
    public void intake(){
        if(!getHatch()){
            fly.set(0.5);
        }
    }

    public void shoot(){
        fly.set(-0.5); 
    }

    public void flyStop(){
        fly.set(0.0); 
    }

    public boolean getBall(){
        return ball_intake.get(); 
    }

    public boolean getHatch(){
        return hatch_pickup.get(); 
    }

    public void setSlot(int slot){
        arm.selectProfileSlot(slot, 0);
    }

    public double getAngle() { // returns angle from ground 

        return 90 - Math.abs(getArmEncoder() * (360.0/4096.0)); //360 degrees per 4096 encoder ticks
    }



    public void updateSmartDashboard(){
        //SmartDashboard.putBoolean("arm up: ", armUp.get()); 
        //SmartDashboard.putBoolean("arm down:", armDown.get()); 
        SmartDashboard.putBoolean("get ball" , getBall()); 
        SmartDashboard.putBoolean("get hatch", getHatch());

        SmartDashboard.putNumber("Arm Encoder:", getArmEncoder());
        SmartDashboard.putNumber("Arm angle: ", getAngle()); 
        SmartDashboard.putNumber("Arm Power:", 0.1 * Math.cos(getAngle())); 
        SmartDashboard.putNumber("Velocity:", getVelocity()); 

        SmartDashboard.putBoolean("Hold arm:", this.hold);
        SmartDashboard.putData("arm holding position:", new toggleArmHold());
    }

    public void displayPID(){
        SmartDashboard.putNumber("Arm Position kP", arm_position_kP);
        SmartDashboard.putNumber("Arm Position kI", arm_position_kI); 
        SmartDashboard.putNumber("Arm Position kD", arm_position_kD); 
        SmartDashboard.putNumber("Arm position setPoint:",  this.position);
    
    }
      // call if tuning PID in execute
    public void tune(){
        // double sdkP = SmartDashboard.getNumber("Arm Position kP", RobotMap.arm_position_kP); 
        // double sdkI = SmartDashboard.getNumber("Arm Position kI", RobotMap.arm_position_kI); 
        // double sdkD = SmartDashboard.getNumber("Arm Position kD", RobotMap.arm_position_kD); 
    
        // double setpoint = SmartDashboard.getNumber("Arm position setPoint:", this.position); 
        double sdkP = preferences.getDouble("Arm Position kP", arm_position_kP); 
        double sdkI = preferences.getDouble("Arm Position kI", arm_position_kI); 
        double sdkD = preferences.getDouble("Arm Position kD", arm_position_kD); 
    
        double setpoint = preferences.getDouble("Arm position setPoint:", this.position); 
    
        if(sdkP != RobotMap.arm_position_kP) {
          arm_position_kP = sdkP; 
          // change slot when doing velocity tuning
          arm.config_kP(RobotMap.ARM_POSITION_SLOT, sdkP);
        }
        if(sdkI != RobotMap.arm_position_kI) {
          arm_position_kI = sdkI;
          // change slot when doing velocity tuning
     
          arm.config_kI(RobotMap.ARM_POSITION_SLOT, sdkI);
    
        }
        if(sdkD != RobotMap.arm_position_kD) {
          arm_position_kD = sdkD;
          // change slot when doing velocity tuning
    
          arm.config_kD(RobotMap.ARM_POSITION_SLOT, sdkD);
    
    
        }
        
        if(setpoint != this.position) this.position = setpoint; 
    
    }

    //arb ff
    public void setPower(double power){
        arm.set(ControlMode.PercentOutput, power); 
    }

    

}

