package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.commands.vision.*;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSink;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 *
 */
public class Vision extends Subsystem {


	public int pipeline = 0; 
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	public UsbCamera cam0; 
	public UsbCamera cam1; 
	CvSink cvSink1; 
	CvSink cvSink2; 
	CvSource outputStream; 
	VideoSink server; 
	
	//Line following
	DigitalInput leftSensor;
	DigitalInput centerSensor;
	DigitalInput rightSensor;
	//public String sensorValues;

	//Ultrasonic
	AnalogInput leftUltra;
	AnalogInput rightUltra;
	double leftUltraDistance;
	double rightUltraDistance;
	//final double mV_T0_mm = 5 / 4.88; // mm/mV


	//Limelight
	NetworkTable table; 
	NetworkTableEntry tx; 
	NetworkTableEntry ty; 
	NetworkTableEntry ta; 
	NetworkTableEntry pipe; 
	public double x, y, area = 0.0; 
	
//	public MjpegServer server = new MjpegServer("Robot camera", 1189); 
	
	public boolean toggle = true; 
	public Vision(){
		CameraServer cs = CameraServer.getInstance();

		cam0 = cs.startAutomaticCapture("cam0", 0); 

		cam0.setResolution(320, 240); 
		cam0.setFPS(15); 

		server = cs.getServer(); 
		server.setSource(cam0); 


		
		
		table = NetworkTableInstance.getDefault().getTable("limelight"); 
		tx = table.getEntry("tx"); 
		ty = table.getEntry("ty"); 
		ta = table.getEntry("ta"); 
		pipe = table.getEntry("getpipe"); 

		leftSensor = new DigitalInput(0);
		centerSensor = new DigitalInput(1);
		rightSensor = new DigitalInput(2);	

		leftUltra = new AnalogInput(RobotMap.LEFT_ULTRA);
		rightUltra = new AnalogInput(RobotMap.RIGHT_ULTRA);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new readNetworkTables());
    	
	}

	public void switchToBall()   {table.getEntry("pipeline").setNumber(1);}
	public void switchToTarget() {table.getEntry("pipeline").setNumber(0);}

	public void changePipeline(int pipeline){
		this.pipeline = pipeline;
		table.getEntry("pipeline").setNumber(pipeline); 
	}
	public double getPipe(){
		return pipe.getDouble(0); 
	}
	public double leftUltra(){
		double mV = leftUltra.getVoltage() * 1000; 
		double mm = mV / 4.88 * 5; 
		return mm / 1000 * 3.28; // returns in feet  
		//return (leftUltra.getVoltage() * 1000) * mV_T0_mm;
	}

	public double rightUltra(){
		// double mV = rightUltra.getVoltage() * 1000; 
		// double mm = mV / 4.88 * 5; 
		// return mm / 1000 * 3.28; // returns in feet	

		return rightUltra.getVoltage(); 
	}
	
	//sensor value flipped: false --> true
	public boolean getLeftPhoto(){
		return !leftSensor.get(); 
	}

	public boolean getMiddlePhoto(){
		return !centerSensor.get(); 
	}

	public boolean getRightPhoto(){
		return !rightSensor.get(); 
	}

	public String getPhotoSensorValues()
	{
		//if using ints 001 --> 1 so using strings
		String left = Integer.toString(boolToInt(leftSensor.get()));
		String center = Integer.toString(boolToInt(centerSensor.get()));
		String right = Integer.toString(boolToInt((rightSensor.get())));
		String sensorValues = left + center + right;
		SmartDashboard.putString("SensorValue:", sensorValues); 

		return sensorValues; 
	}

	public int boolToInt(boolean b) { return b ? 1 : 0; }
    
    public void getValues() {
    	x = tx.getDouble(0.0); 
    	y = ty.getDouble(0.0); 
    	area = ta.getDouble(0.0); 
    	SmartDashboard.putNumber("LimelightX", x); 
    	SmartDashboard.putNumber("LimelightY",  y); 
    	SmartDashboard.putNumber("LimelightArea",  area); 
    }
    
    public void toggleCamera(){
    	if(toggle){
    		server.setSource(cam1);
    	} else{
    		server.setSource(cam0); 
    	}
    	toggle = !toggle; 
    }
    
    public void updateSmartDashboard(){
   		SmartDashboard.putNumber("Left Ultra:", leftUltra()); 
		SmartDashboard.putNumber("Right Ultra:",  rightUltra()); 
		SmartDashboard.putNumber("Pipeline: ", getPipe()); 
		// for testing, to make sure buttons work 
		SmartDashboard.putNumber("FAKE PIPLINE:", this.pipeline); 

		SmartDashboard.putBoolean("Left photo", getLeftPhoto());
		SmartDashboard.putBoolean("Center photo", getMiddlePhoto()); 
		SmartDashboard.putBoolean("Right photo", getRightPhoto()); 
   	//SmartDashboard.putNumber("LimelightArea",  area); 
    }
}

