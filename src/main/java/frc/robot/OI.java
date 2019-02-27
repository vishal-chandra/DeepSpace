package frc.robot;

import frc.robot.commands.arm.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.elevator.*;
import frc.robot.commands.vision.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

//import frcrobot.commands.ExampleCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	// public static Joystick right = new Joystick(1);
	// 	Button r1 = new JoystickButton(right, 1);
	// 	Button r2 = new JoystickButton(right, 2);
	// 	Button r3 = new JoystickButton(right, 3);
	// 	Button r4 = new JoystickButton(right, 4);
	// 	Button r5 = new JoystickButton(right, 5);
	// 	Button r6 = new JoystickButton(right, 6);
	// 	Button r7 = new JoystickButton(right, 7);
	// 	Button r8 = new JoystickButton(right, 8);
	// 	Button r9 = new JoystickButton(right, 9);
	// 	Button r10 = new JoystickButton(right, 10);
	// 	Button r11 = new JoystickButton(right, 11);

	// public static Joystick left = new Joystick(2);
	// 	Button l1 = new JoystickButton(left, 1);
	// 	Button l2 = new JoystickButton(left, 2);
	// 	Button l3 = new JoystickButton(left, 3);
	// 	Button l4 = new JoystickButton(left, 4);
	// 	Button l5 = new JoystickButton(left, 5);
	// 	Button l6 = new JoystickButton(left, 6);
	// 	Button l7 = new JoystickButton(left, 7);
	// 	Button l8 = new JoystickButton(left, 8);
	// 	Button l9 = new JoystickButton(left, 9);
	// 	Button l10 = new JoystickButton(left, 10);
	// 	Button l11 = new JoystickButton(left, 11);
	public static Joystick controller = new Joystick(1);
		Button c1 = new JoystickButton(controller, 1);
		Button c2 = new JoystickButton(controller, 2);
		Button c3 = new JoystickButton(controller, 3);
		Button c4 = new JoystickButton(controller, 4);
		Button c5 = new JoystickButton(controller, 5);
		Button c6 = new JoystickButton(controller, 6);
		Button c7 = new JoystickButton(controller, 7);
		Button c8 = new JoystickButton(controller, 8);
		Button c9 = new JoystickButton(controller, 9);
		Button c10 = new JoystickButton(controller, 10);
		Button c11 = new JoystickButton(controller, 11);
		Button c12 = new JoystickButton(controller, 12);
		
	public static XboxController xbox = new XboxController(0);
		//Button x1 = new JoystickButton(xbox, 1); 
		 

//	public static Joystick xbox = new Joystick(2);
		Button x1 = new JoystickButton(xbox, 1);
		Button x2 = new JoystickButton(xbox, 2);
		Button x3 = new JoystickButton(xbox, 3);
		Button x4 = new JoystickButton(xbox, 4);
		Button x5 = new JoystickButton(xbox, 5);
		Button x6 = new JoystickButton(xbox, 6);
		Button x7 = new JoystickButton(xbox, 7);
		Button x8 = new JoystickButton(xbox, 8);
		Button x9 = new JoystickButton(xbox, 9);
		Button x10 = new JoystickButton(xbox, 10);
		
		// 1: A
		// 2: B
		// 3: X 
		// 4: Y 
		// 5: left bumper 
		// 6: right bumper 
		// 7: back
		// 8: start
		// 9: left joystick 
		// 10: right joystick 
	public OI(){
		
	
		/* logitech controller */


		// r1.toggleWhenPressed(new driveBaseSwitchDirections());
		//c1.whileHeld(new rotateClockwise());
		//c2.whileHeld(new rotateCounterClockwise());
		//c3.whenPressed(new turnRight90()); 
		//c4.whenPressed(new autoAlign()); 
		//c7.whileHeld(new driveStraight(-0.8, -0.8));
		//x1.whileHeld(new driveStraight(0.6, 0.6));
		//c6.whenPressed(new followTarget());
		//c9.whenPressed(new changePipeline(0));
		//c10.whenPressed(new changePipeline(1));

		c1.whileHeld(new armPower(Robot.arm.HATCH_HIGH_POWER)); 
		c2.whileHeld(new armPower(Robot.arm.HATCH_LOW_POWER)); 

		//c6.whileHeld(new armPower(Robot.arm.BALL_HIGH_POWER)); 

		c8.whileHeld(new armPower(Robot.arm.NO_HATCH_LOW_POWER)); 

		//c9.whenPressed(new elevatorLIDAR(Robot.elevator.HATCH_PICKUP_LOW_LIDAR));
		//c10.whenPressed(new elevatorLIDAR(Robot.elevator.HATCH_PICKUP_RAISE_LIDAR)); 
		
		 
//		c8.whenPressed(new streamCameras());
		//c6.whileHeld(new armPower(Robot.arm.BALL_HIGH_POWER)); 

		//c8.whileHeld(new armPower(Robot.arm.NO_HATCH_LOW_POWER)); 
		c6.whenPressed(new setArmPosition(800)); 
		c4.whileHeld(new flySuck()); 
		c3.whileHeld(new flyShoot());
		c5.whileHeld(new armRaise());
		c7.whileHeld(new armLower()); 
		//c6.whileHeld(new followTape());
		
		
		
	
	}
}
