/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.moveElevatorJoystick;
import frc.robot.commands.setElevator;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  WPI_TalonSRX elevator; 
  public double position; 
  double arbfeedfwd; 


  public Elevator(double position){
    this.position = position; 

    elevator = new WPI_TalonSRX(RobotMap.ELEVATOR); 

    // encoder stuff 
    elevator.configFactoryDefault();
    elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    //elevator.setSensorPhase(true);
    
    elevator.getSensorCollection().setQuadraturePosition(0, 30);
    
    elevator.configNominalOutputForward(0, 30);
    elevator.configNominalOutputReverse(0, 30);
    elevator.configPeakOutputForward(0.5, 30); 
    elevator.configPeakOutputReverse(-0.5, 30);

    // POSITION: 0 
    this.setPID(RobotMap.ELEVATOR_POSITION_SLOT, RobotMap.elevator_position_kF, 
      RobotMap.elevator_position_kP, RobotMap.elevator_position_kI, 
      RobotMap.elevator_velocity_kD); 
    // VELOCITY: 1
    this.setPID(RobotMap.ELEVATOR_VELOCITY_SLOT, RobotMap.elevator_velocity_kF,
      RobotMap.elevator_velocity_kP, RobotMap.elevator_velocity_kI,
      RobotMap.elevator_velocity_kD);

    

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    //setDefaultCommand(new setElevator());
    
    // moveElevatorJoystick is for finding arbitrary feed forwardd to use with position control
    setDefaultCommand(new moveElevatorJoystick()); 
  }



  /*
  Methods: 
    lift: up/down 
    set position 

    get encoder 
    reset encoder 

  */

  public void setPosition(double setpoint){
     // elevator.set(ControlMode.position, setpoint, Demandtype.ArbitraryFeedForward, arbfeedfwd)

      elevator.set(ControlMode.Position, setpoint); 
  }

  public void setSpeed(double speed){
    elevator.set(ControlMode.Velocity, speed); 
  }

  public void setPower(double power){
    elevator.set(ControlMode.PercentOutput, power); 
  }

  public void raiseElevator(){
    elevator.set(ControlMode.PercentOutput, 0.2); 

  }

  public void lowerElevator(){
    elevator.set(ControlMode.PercentOutput, -0.1); 


  }
  public void resetEncoder(){
    elevator.getSensorCollection().setQuadraturePosition(0, 10);

  }

  public double getPosition(){
    return elevator.getSensorCollection().getQuadraturePosition();
  }

  public double getSpeed(){
    return elevator.getSensorCollection().getQuadratureVelocity();  
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


    //displayPID();
    
  }
  // call if tuning PID in initialize
  public void displayPID(){
    SmartDashboard.putNumber("Elevator Position kP", RobotMap.elevator_position_kP);
    SmartDashboard.putNumber("Elevator Position kI", RobotMap.elevator_position_kI); 
    SmartDashboard.putNumber("Elevator Position kD", RobotMap.elevator_position_kD); 
    SmartDashboard.putNumber("Elevator position setPoint:",  this.position); 

  }
  // call if tuning PID in execute
  public void tune(){
    double sdkP = SmartDashboard.getNumber("Elevator Position kP", RobotMap.elevator_position_kP); 
    double sdkI = SmartDashboard.getNumber("Elevator Position kI", RobotMap.elevator_position_kI); 
    double sdkD = SmartDashboard.getNumber("Elevator Position kD", RobotMap.elevator_position_kD); 

    double setpoint = SmartDashboard.getNumber("Elevator position setPoint:", this.position); 

    if(sdkP != RobotMap.elevator_position_kP) {
      RobotMap.elevator_position_kP = sdkP; 
      // change slot when doing velocity tuning
      elevator.config_kP(RobotMap.ELEVATOR_POSITION_SLOT, sdkP);
    }
    if(sdkI != RobotMap.elevator_position_kI) {
      RobotMap.elevator_position_kI = sdkI;
      // change slot when doing velocity tuning
 
      elevator.config_kI(RobotMap.ELEVATOR_POSITION_SLOT, sdkI);

    }
    if(sdkD != RobotMap.elevator_position_kD) {
      RobotMap.elevator_position_kD = sdkD;
      // change slot when doing velocity tuning

      elevator.config_kD(RobotMap.ELEVATOR_POSITION_SLOT, sdkD);


    }
    
    if(setpoint != this.position) this.position = setpoint; 

  }
}
