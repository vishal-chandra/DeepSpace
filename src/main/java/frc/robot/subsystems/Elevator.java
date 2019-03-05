/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.elevator.*;
import frc.robot.LIDAR;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public LIDAR lidar;
  WPI_TalonSRX elevator; 
  public double position; 
  double arbfeedfwd; 

  public DigitalInput elevator_down; 
  public DigitalInput carriage_up; 
  public DigitalInput stage2_up;

  public double MAX_ENCODER_POSITION = 48119; 
  public double HATCH_PICKUP_LOW_LIDAR = 27; 
  public double HATCH_PICKUP_RAISE_LIDAR = 37; 

  public double mm_kP = 0.00; 
  public double mm_kI = 0.000; 
  public double mm_kD = 0.000; 
  public double mm_kF = 0.0; 

  public int CRUISE_VELOCITY = 0; 
  public int ACCELERATION = 0; 
  public int kTimeoutMs = 5; 


  public Elevator(double position){
    this.position = position; 

    elevator = new WPI_TalonSRX(RobotMap.ELEVATOR);

    // encoder stuff 
    elevator.configFactoryDefault();
    elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    //elevator.setSensorPhase(true);
    elevator.setInverted(true); 
    
    elevator.getSensorCollection().setQuadraturePosition(0, 30);  
    
    elevator.configNominalOutputForward(0, 30);
    elevator.configNominalOutputReverse(0, 30);
    elevator.configPeakOutputForward(1.0, 30); 
    elevator.configPeakOutputReverse(-1.0, 30);

    elevator.configMotionCruiseVelocity(CRUISE_VELOCITY, kTimeoutMs); 
    elevator.configMotionAcceleration(ACCELERATION, kTimeoutMs); 

    // MOTION MAGIC: 0 
    this.setPID(0, mm_kF, mm_kP, mm_kI, mm_kD); 


    carriage_up = new DigitalInput(RobotMap.CARRIAGE_UP_SWITCH); 
    stage2_up = new DigitalInput(RobotMap.STAGE2_UP_SWITCH);
    elevator_down = new DigitalInput(RobotMap.ELEVATOR_DOWN_SWITCH); 

    lidar = new LIDAR();
    lidar.startMeasuring();
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new setElevator());
    
    // moveElevatorJoystick is for finding arbitrary feed forwardd to use with position control
    setDefaultCommand(new moveElevatorJoystick()); //arb ff
    //setDefaultCommand(new elevatorLIDAR());
  }



  /*
  Methods: 
    lift: up/down 
    set position 

    get encoder 
    reset encoder 

  */

  public double getFeedForward(){
    double feedForward = 0.0; 
    
    
    return feedForward; 
  
  }
  public void setPosition(double setpoint){
     double feedForward = getFeedForward(); 
     elevator.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, feedForward); 

      //elevator.set(ControlMode.Position, setpoint); 
  }

  public void setSpeed(double speed){
    elevator.set(ControlMode.Velocity, speed); 
  }

  public void setPower(double power){
    elevator.set(ControlMode.PercentOutput, power); 
  }

  public void raiseElevator(){
    //if(!(carriage_up.get() && stage2_up.get())){
      elevator.set(ControlMode.PercentOutput, 0.5); 
    //}

  }

  public void lowerElevator(){
    //if(!elevator_down.get()){
      elevator.set(ControlMode.PercentOutput, -0.5); 
    //}

  }

  public void stopElevator(){
    elevator.set(ControlMode.PercentOutput, 0.0); 
  }
  public void resetEncoder(){
    elevator.getSensorCollection().setQuadraturePosition(0, 10);

  }

  public double getPosition(){
    return elevator.getSelectedSensorPosition();
  }

  public double getSpeed(){
    return elevator.getSelectedSensorVelocity();  
  }

  public void setPID(int slot, double kF, double kP, double kI, double kD){
    elevator.config_kF(slot, kF, 30); 
    elevator.config_kP(slot, kP, 30); 
    elevator.config_kI(slot, kI, 30); 
    elevator.config_kD(slot, kD, 30); 
  }

  public void resetElevator(){
    elevator.getSensorCollection().setQuadraturePosition(0, 30);
  }

  public void setSlot(int slot){
    elevator.selectProfileSlot(slot, 0);
  }
  public void updateSmartDashboard(){
    SmartDashboard.putNumber("Elevator Position:", getPosition()); 
    SmartDashboard.putNumber("Elevator Speed: ", getSpeed());
    SmartDashboard.putNumber("Lidar Distance: ", lidar.getDistance());

    SmartDashboard.putBoolean("Carriage up:", carriage_up.get()); 
    SmartDashboard.putBoolean("Second Stage up:", stage2_up.get());
    SmartDashboard.putBoolean("Elevator down:", elevator_down.get()); 
    //displayPID();
    
  }  
}
