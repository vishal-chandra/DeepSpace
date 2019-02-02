package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	
	// MOTOR PORTS 
	public static final int TOP_LEFT_DRIVETRAIN_MOTOR = 1; 
	public static final int BOTTOM_LEFT_DRIVETRAIN_MOTOR = 1; 

	public static final int TOP_RIGHT_DRIVETRAIN_MOTOR = 0; 
	public static final int BOTTOM_RIGHT_DRIVETRAIN_MOTOR = 0; 
	
	public static final int LEFT_ENCODER_ONE = 0; 
	public static final int LEFT_ENCODER_TWO = 1; 
	public static final int RIGHT_ENCODER_ONE = 2; 
	public static final int RIGHT_ENCODER_TWO = 3; 

	public static final int ELEVATOR = 0; 

	//ball actuator thing 
	public static final int ARM_ACTUATOR = 0; 
	//fly wheels 
	public static final int FLY = 0; 

	
	public static final double kP = 0.5; 
	public static final double kI = 0.3; 
	public static final double kD = 0.4; 
	
	public static final double TOLERANCE = 0.5; 
	public static final double ALIGN_STEADY_TIME = 100; 
	
	public static final double SPEED = 0.50; 
	
	public static boolean open = true; 
	
	public static boolean camOneStart = true; 
	public static boolean camTwoStart = true; 
	
	public static int ARM_MOTOR = 1; 

}
