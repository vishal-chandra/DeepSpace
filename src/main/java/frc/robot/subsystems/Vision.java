package frc.robot.subsystems;

import frc.robot.commands.readNetworkTables;

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
	public String sensorValues;

	//Ultrasonic
	AnalogInput leftUltra;
	AnalogInput rightUltra;
	double leftUltraDistance;
	double rightUltraDistance;
	final double mV_T0_mm = 5 / 4.88; // mm/mV


	//Limelight
	NetworkTable table; 
	NetworkTableEntry tx; 
	NetworkTableEntry ty; 
	NetworkTableEntry ta; 
	public double x, y, area = 0.0; 
	
//	public MjpegServer server = new MjpegServer("Robot camera", 1189); 
	
	public boolean toggle = true; 
	public Vision(){
//		CameraServer cs = CameraServer.getInstance(); 
//		
//		cam0 = cs.startAutomaticCapture("cam0", 0); 
//		cam1 = cs.startAutomaticCapture("cam1", 1); 
//
//		cs.addCamera(cam0); 
//		cs.addCamera(cam1); 
//		
//		
////		cam0 = new UsbCamera("cam0", 0); 
////		cam1 = new UsbCamera("cam1", 1);  
////		
//		cam0.setResolution(320, 240); 
//		cam1.setResolution(320, 240); 
//		
//		cam0.setFPS(15); 
//		cam1.setFPS(15); 
//		
//		server = cs.getServer(); 
//		server.setSource(cam0); 
		
		table = NetworkTableInstance.getDefault().getTable("limelight"); 
		tx = table.getEntry("tx"); 
		ty = table.getEntry("ty"); 
		ta = table.getEntry("ta"); 

		leftSensor = new DigitalInput(0);
		centerSensor = new DigitalInput(1);
		rightSensor = new DigitalInput(2);	

		leftUltra = new AnalogInput(1);
		rightUltra = new AnalogInput(2);
	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
    	//setDefaultCommand(new readNetworkTables());
    	
	}

	public void switchToBall()   {table.getEntry("pipeline").setNumber(1);}
	public void switchToTarget() {table.getEntry("pipeline").setNumber(0);}

	public void changePipeline(int pipeline){
		table.getEntry("pipeline").setNumber(pipeline); 
	}

	public double leftUltra(){
		return (leftUltra.getVoltage() * 1000) * mV_T0_mm;
	}

	public double rightUltra(){
		return (rightUltra.getVoltage() * 1000) * mV_T0_mm; 
	}
	
	public void getPhotoSensorValues()
	{
		//if using ints 001 --> 1 so using strings
		String left = Integer.toString(boolToInt(leftSensor.get()));
		String center = Integer.toString(boolToInt(centerSensor.get()));
		String right = Integer.toString(boolToInt((rightSensor.get())));
		sensorValues = left + center + right;
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
   	//SmartDashboard.putNumber("LimelightArea",  area); 
    }
}

