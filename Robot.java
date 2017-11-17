/**
 * This code is the code made for the 2017 FRIST FRC. This team is 6423. 
 * Authors: Rohan Rajagopalan, Eddie Yan, Chukwudumebi Joshua Obi
 * 
 */



package org.usfirst.frc.team6423.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 **/
public class Robot extends IterativeRobot 
{
	final String defaultAuto = "Default";
	final String customAuto = "My Auto";
	String autoSelected;
	SendableChooser<String> chooser = new SendableChooser<>();
	
	RobotDrive myDrive;
	Spark frontLeft, frontRight, rearLeft, rearRight;
	static TalonSRX ropeClimber;
	Joystick leftStick, rightStick; 
	JoystickButton ropeUp;
	JoystickButton accelerate;
	//JoystickButton slowDown;
	static Timer timer;
	// new thing VV
	static boolean fast = false;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit()
	{
		chooser.addDefault("Default Auto", defaultAuto);
		chooser.addObject("My Auto", customAuto);
		
		// Objects for the Spark Motor controllers. Argument corresponds to the sock the controller is connected to. 
		frontLeft = new Spark(1);
		frontRight = new Spark(3);
		rearLeft = new Spark(0);
		rearRight = new Spark(2);
		
		frontLeft.setSpeed(0.7);
		frontLeft.enableDeadbandElimination(true);
		frontRight.setSpeed(0.7);
		frontRight.enableDeadbandElimination(true);
		rearLeft.setSpeed(0.7);
		rearLeft.enableDeadbandElimination(true);
		rearRight.setSpeed(0.7);
		rearRight.enableDeadbandElimination(true);
		
		leftStick = new Joystick(0); //Joystick objects. Buttons will use these 
		rightStick = new Joystick(1);
		
		ropeUp = new JoystickButton(rightStick, 1);// Make sure button objects are assigned different numbers. 
		accelerate = new JoystickButton(leftStick,2); // If you don't it will not work. 

		ropeClimber = new TalonSRX(4);
		ropeClimber.setSpeed(0);
		ropeClimber.enableDeadbandElimination(true);
		
		timer = new Timer();
		
		myDrive = new RobotDrive(frontLeft, rearLeft, frontRight, rearRight);
		myDrive.setSensitivity(0.7);
		//In this is particular form we have selected the camera the webdashbord has named called camo0. We have set its port to 0 (Java does this automatically but we're going to do it manuallly) 
		
		CameraServer.getInstance().startAutomaticCapture();
		CameraServer.getInstance().startAutomaticCapture(); 
	
		myDrive.setMaxOutput(.4);
		myDrive.setExpiration(0.1);
		
     }
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override	
	public void autonomousInit() 
	{
		myDrive.setSafetyEnabled(true);
		 	   timer.reset();

		    // Start timer
		    timer.start();
		
	}
		
	//This function is called periodically during autonomous
	 @Override
	public void autonomousPeriodic()
	{
		Scheduler.getInstance().run();
			while (isAutonomous() && isEnabled())
		{
			// > 0 turns right... < 0 turns left
			//middle 
			/*
			if(timer.get() < 2.0)
			{
				myDrive.drive(-.5, 0.0);
			}
			else 
			{
				myDrive.drive(0.0, 0.0);
				timer.stop();
			}	
			*/
			/*
			// right side... drives straight
			if(timer.get() < 5.0)
			{
				myDrive.drive(-.5, 0.0);  
			}
			else
			{
				timer.stop();
				myDrive.drive(0.0, 0.0);
			}
			*/
			// left side code... turns around
			
			if (timer.get() < 6.0)
			{
				myDrive.drive(-.5, 0.0);
			}
			else if (timer.get() >= 6.0 && timer.get() < 9.8)
			{
				myDrive.drive(-.5, -1.0);
			}
			else if (timer.get() >= 9.8 && timer.get() < 10.8)
			{
				myDrive.drive(-.5, 0.0);
			}
			else
			{
				timer.stop();
				myDrive.drive(0.0, 0.0);
			}
			
			
			
		}
		/*switch (autoSelected) 
		{
		case customAuto:
			// Put custom auto code here
			break;
		case defaultAuto:
		default:
			// Put default auto code here
			break;
		}
		*/	
	}
	
	@Override
	public void teleopInit() {
		
		Scheduler.getInstance().run();
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		while ( isOperatorControl() && isEnabled())
		{
			myDrive.setSafetyEnabled(false);
			myDrive.tankDrive(leftStick, rightStick);
			
			buttonSpeedUp(accelerate);
			Timer.delay(.005);
			ropeClimb(ropeUp);		
		}
	}
     /**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() 
	{
		LiveWindow.run();	
	}
	
	public static void ropeClimb(JoystickButton button1) 
	{
		while( button1.get() )
		{
			ropeClimber.set(-1);
		}
	  ropeClimber.set(0);
	}
	
	public void buttonSpeedUp(JoystickButton button2) 
	{
	
		while( button2.get() ) {  
			
			frontLeft.setSpeed(-0.8);
			frontLeft.enableDeadbandElimination(true);
			frontRight.setSpeed(0.8);
			frontRight.enableDeadbandElimination(true);
			rearLeft.setSpeed(-0.8);
			rearLeft.enableDeadbandElimination(true);
			rearRight.setSpeed(0.8);
			rearRight.enableDeadbandElimination(true);
		     }
	}    
}