/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  WPI_TalonSRX arm_actuator; 
  WPI_TalonSRX fly_wheels; 


  public Intake(){
    arm_actuator = new WPI_TalonSRX(RobotMap.ARM_ACTUATOR);
    fly_wheels = new WPI_TalonSRX(RobotMap.FLY);

    // set encoder stuff 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // Methods: 
  /*

    Arm-up/down 

    flywheels - forward/reverse 

    get encoder values 

    reset encoder 

    setPosition 
  */
}
