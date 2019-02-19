package frc.robot.commands.vision;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class followTape extends Command 
{

  double angle = 0;
  double speed = 0.4;
  String value;
  boolean finished = false; 

  public followTape() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);
    requires(Robot.vision);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      value = Robot.vision.getPhotoSensorValues();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      value = Robot.vision.getPhotoSensorValues();
      //value = Robot.vision.sensorValues;
      SmartDashboard.putString("Sensor values", value); 

      if(value.equals("100")) angle = -0.15; //slight left
      else if(value.equals("110")) angle = -0.07; //very slight left

      else if(value.equals("001")) angle = 0.15; //slight right
      else if(value.equals("011")) angle = 0.7; //very slight right

      else if(value.equals("010")) angle = 0; //go straight
      else if(value.equals("101") || value.equals("111") || value.equals("000")) 
      {
          angle = 0; 
          speed = 0; //stop, there's been an error
          finished = true; 
      }

      Robot.driveTrain.curavtureDrive(speed, angle);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
      Robot.driveTrain.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
      Robot.driveTrain.stop();
      end(); 
  }
}
