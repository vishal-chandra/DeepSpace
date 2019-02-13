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
	public static  int BACK_LEFT_MOTOR = 1; 
	public static  int FRONT_LEFT_MOTOR = 2;
	public static  int FRONT_RIGHT_MOTOR = 3; 
	public static  int BACK_RIGHT_MOTOR = 4; 
	
	
	public static  int LEFT_ENCODER_ONE = 0; 
	public static  int LEFT_ENCODER_TWO = 1; 
	public static  int RIGHT_ENCODER_ONE = 2; 
	public static  int RIGHT_ENCODER_TWO = 3; 

	public static  int ELEVATOR = 0; 

	//ball actuator thing 
	public static  int ARM_ACTUATOR = 0; 
	//fly wheels 
	public static  int FLY = 0; 

	public static  int ELEVATOR_POSITION_SLOT = 0; 
	public static  int ELEVATOR_VELOCITY_SLOT = 1; 
	public static  double ELEVATOR_ARBFEEDFWD = 0.0; 

	// TODO: TUNE PID
	// Elevator manual velocity PID 
	public static double elevator_velocity_kF = 0.0;
	public static double elevator_velocity_kP = 0.0; 
	public static double elevator_velocity_kI = 0.0; 
	public static  double elevator_velocity_kD = 0.0; 

	// elevator position PID 
	public static  double elevator_position_kF = 0.0; 
	public static  double elevator_position_kP = 0.0; 
	public static  double elevator_position_kI = 0.0; 
	public static  double elevator_position_kD = 0.0; 

	public static  int ARM_POSITION_SLOT = 0; 
	public static  int ARM_VELOCITY_SLOT = 1;

	// TODO: TUNE PID
	// arm position PID 
	public static  double arm_position_kF = 0.0; 
	public static  double arm_position_kP = 0.0; 
	public static  double arm_position_kI = 0.0; 
	public static  double arm_position_kD = 0.0;

	public static  double arm_velocity_kF = 0.0; 
	public static  double arm_velocity_kP = 0.0; 
	public static  double arm_velocity_kI = 0.0; 
	public static  double arm_velocity_kD = 0.0;

	public static  double kP = 0.5; 
	public static  double kI = 0.3; 
	public static  double kD = 0.4; 
	
	public static  double TOLERANCE = 0.5; 
	public static  double ALIGN_STEADY_TIME = 100; 
	
	public static  double SPEED = 0.50; 
	
	public static boolean open = true; 
	
	public static boolean camOneStart = true; 
	public static boolean camTwoStart = true; 
	
	public static int ARM_MOTOR = 1; 

}
