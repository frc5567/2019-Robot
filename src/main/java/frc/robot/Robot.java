/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.Drivetrain;
import frc.robot.Controller;
import frc.robot.Climber;
import frc.robot.NavX;
import frc.robot.Elevator.State;
import frc.robot.Elevator;
import frc.robot.HatchMech;
import frc.robot.AutoCommands;
import frc.robot.TeleopCommands;
import frc.robot.GamePad;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	// Declare drivetrain
	Drivetrain m_drivetrain;
	Pathing m_pather;

	// Declare Pilot XBox Controller
	Controller m_pilotController;

	// Declares xbox controller for co-pilot
	// Used for testing, gamepad will be used in comp
	Controller m_copilotController;

	GamePad m_copilotGamepad;

	// Declare climbing mechanisms for front and back climbers
	Climber m_frontClimber;
	Climber m_backClimber;

	// Declare NavX
	NavX m_ahrs;

	// Declares Elevator
	Elevator m_elevator;

	// Declare Auto Commands class for auto and auto assist commands
	AutoCommands autoCommands;
	// Declare Teleop commands for pilot controller methods
	TeleopCommands teleopCommands;

	// Declare Dashboard and Dashboard data bus
	DashboardData m_dataStream;
	CustomDashboard m_roboDash;

	// Declaring the USB Camera
	UsbCamera camera;
	HatchMech m_hatchMech;

	// Declare our duino communication port
	// private DuinoToRioComms m_duinoToRio;
	// private DuinoCommStorage m_pkt;

	Robot() {

		// Instanciates drivetrain, driver controllers, climbers, and elevator
		m_drivetrain = new Drivetrain(m_ahrs);
		m_pilotController = new Controller(RobotMap.PILOT_CONTROLLER_PORT);

		m_elevator = new Elevator();
		
		m_copilotController = new Controller(RobotMap.COPILOT_CONTROLLER_PORT);
		m_copilotGamepad = new GamePad(RobotMap.COPILOT_CONTROLLER_PORT);

		m_frontClimber = new Climber(RobotMap.FRONT_CLIMBER_MOTOR_PORT, RobotMap.FRONT_CLIMBER_LIMIT_TOP_PORT, RobotMap.FRONT_CLIMBER_LIMIT_BOTTOM_PORT);
		m_backClimber = new Climber(RobotMap.BACK_CLIMBER_MOTOR_PORT, RobotMap.BACK_CLIMBER_LIMIT_TOP_PORT, RobotMap.BACK_CLIMBER_LIMIT_BOTTOM_PORT);
		m_elevator = new Elevator();
		m_elevator.elevatorPIDConfig();

		// Instantiate our duino to rio communication port
		// m_duinoToRio = new DuinoToRioComms();
		
		try {
			m_ahrs = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
		}

		m_drivetrain = new Drivetrain(m_ahrs);

		m_pather = new Pathing(m_drivetrain, m_ahrs);

		autoCommands = new AutoCommands(m_drivetrain, m_ahrs, m_elevator, m_frontClimber, m_backClimber);
		teleopCommands = new TeleopCommands(m_pilotController, m_copilotGamepad, m_drivetrain, m_elevator, m_frontClimber, m_backClimber, m_hatchMech);
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		//	Sets up the camera and inits the camera server
//		camera = CameraServer.getInstance().startAutomaticCapture();
//		camera.setResolution(160, 120);
//		camera.setFPS(1);
	}

	/**
	 * This function is called every robot packet, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the getString line to get the
	 * auto name from the text box below the Gyro
	 *
	 * <p>
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {

	}

	/**
	 * This function is called once before the operator control period starts
	 */
	@Override
	public void teleopInit() {
		m_drivetrain.talonDriveConfig();
		m_pather.resetFlags();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		// Test drivetrain included, uses Left stick Y for speed, Right stick X for
		// turning, quick turn is auto-enabled at low speed


		// PID based sample talon arcade drive
		// m_drivetrain.talonArcadeDrive(m_pilotController.getRightTrigger() - m_pilotController.getLeftTrigger(), m_pilotController.getLeftStickX());
		if(m_pilotController.getYButton()) {
			/*System.out.println*/m_pather.pathToTarget();
//			System.out.println("Why are buttons?");
//			System.out.println("LeftEnc\t" + m_drivetrain.getLeftDriveEncoderPosition());
//			System.out.println("RightEnc\t" + m_drivetrain.getRightDriveEncoderPosition());
		}
		else if (m_pilotController.getXButton()) {
			m_pather.secondHalfPath();
		}
		else {
			m_drivetrain.talonArcadeDrive(m_pilotController.getLeftStickY(), m_pilotController.getRightStickX());
		}
		
		if (m_pilotController.getAButtonReleased()) {
			m_ahrs.zeroYaw();
		}
		if (m_pilotController.getBButtonReleased()) {
			m_ahrs.flipOffset();
		}
		if (m_pilotController.getBumper(Hand.kRight)) {
			m_pather.resetFlags();
		}
		m_pilotController.setRumble(RumbleType.kLeftRumble, 0);
		m_pilotController.setRumble(RumbleType.kRightRumble, 0);

		// Prints yaw and if offset is applied to console
		System.out.println(m_ahrs.getOffsetYaw() + "\t\t" + m_ahrs.getOffsetStatus());
	}

	/**
	 * This function is called once before starting test mdoe
	 */
	@Override
	public void testInit() {

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		/*
		 * // Code for testing comms with arduino if
		 * (m_pilotController.getAButtonReleased()) { // Assigns return value. Checking
		 * NaN should occur here m_degToTarget = m_duinoToRio.getDegToTarget(); if
		 * (m_degToTarget.isNaN()){ System.out.println("No number returned"); } else {
		 * System.out.println("degToTarget: " + m_degToTarget); //m_pkt.degTargetHigh =
		 * degToTarget; }
		 * 
		 * } else if (m_pilotController.getBButtonReleased()) { // Assigns return value.
		 * Checking NaN should occur here m_distToTarget =
		 * m_duinoToRio.getDistToTarget(); if (m_distToTarget.isNaN()){
		 * System.out.println("No number returned"); } else {
		 * System.out.println("distToTarget: " + m_distToTarget); //m_pkt.distTargetHigh
		 * = distToTarget; }
		 * 
		 * 
		 * } else if (m_pilotController.getXButtonReleased()) { // Assigns return value.
		 * Checking NaN should occur here m_angleToCenter =
		 * m_duinoToRio.getAngleToCenter(); if (m_distToTarget.isNaN()){
		 * System.out.println("No number returned"); } else {
		 * System.out.println("angleToCenter: " + m_angleToCenter);
		 * //m_pkt.distTargetHigh = distToTarget; }
		 * 
		 * } else if (m_pilotController.getYButtonReleased()) { // Assigns return value.
		 * Checking NaN should occur here m_lowPosition = m_duinoToRio.getLowPosition();
		 * if (m_lowPosition.isNaN()){ System.out.println("No number returned"); } else
		 * { System.out.println("lowPosition: " + m_lowPosition); //m_pkt.distTargetHigh
		 * = distToTarget; }
		 * 
		 * }
		 */


		// BIG TEST CODE
		
		// Stuff from Teleop
		// Test drivetrain included, uses Left stick Y for speed, Right stick X for
		// turning, quick turn is auto-enabled at low speed
		// m_drivetrain.curvatureDrive(m_pilotController.getLeftStickY(), m_pilotController.getRightStickX());

		// Zeros yaw if 'A' is pressed, and adds 180 degree offset if 'B' is pressed
		// if (m_pilotController.getAButtonReleased()) {
		// 	m_ahrs.zeroYaw();
		// }
		// if (m_pilotController.getBButtonReleased()) {
		// 	m_ahrs.flipOffset();
		// }

		// Prints yaw and if offset is applied to console
		//System.out.println(m_ahrs.getOffsetYaw() + "\t\t" + m_ahrs.getOffsetStatus());

		// New Stuff
		// Elevator controls, triggers are for testing as of 2/16
		//System.out.println(m_pilotController.getLeftTrigger() - m_pilotController.getRightTrigger());
		//m_elevator.moveRaw(m_pilotController.getLeftTrigger() - m_pilotController.getRightTrigger());

		// [NOTE] Negative power moves the elevator up, but the encoder will still tic
		// positive. This is due to the way the string is wound on the winch
		// Follow up: This is no longer quite true. So long as we call the elevator PID config, the motor will be inverted, thus positive should be up
		if(m_pilotController.getAButton()) {
			m_elevator.moveRaw(-.4);
		}
		else if (m_pilotController.getBButton()) {
			m_elevator.moveRaw(.4);
		}
		else if (m_pilotController.getXButton()) {
			m_elevator.elevatorPIDDrive(State.HATCH_L1);
			//m_elevator.moveToPosition(m_pilotController.getXButton() , State.HATCH_L1);
		}
		else if (m_pilotController.getYButton()) {
			m_elevator.elevatorPIDDrive(State.HATCH_L2);
			//m_elevator.moveToPosition(m_pilotController.getYButton() , State.HATCH_L2);
		}
		else if (m_pilotController.getBumper(Hand.kRight)) {
			m_elevator.elevatorPIDDrive(State.HATCH_L3);
			//m_elevator.moveToPosition(m_pilotController.getBumper(Hand.kRight) , State.HATCH_L3);
		}
		else if (m_pilotController.getBumper(Hand.kLeft)) {
			m_elevator.elevatorPIDDrive(State.LEVEL_ZERO);
		}
		else if (m_pilotController.getStartButton()) {
			m_elevator.m_elevatorEncoder.setQuadraturePosition(0, 0);
		}
		else {
			m_elevator.moveRaw(0);
		}

		if (m_copilotController.getAButton()) {
			m_frontClimber.raiseClimber();
		}
		else if (m_copilotController.getBButton()) {
			m_frontClimber.lowerClimber();
		}
		else {
			m_frontClimber.setClimber(0.0);
		}

		if (m_copilotController.getXButton()) {
			m_backClimber.raiseClimber();
		}
		else if (m_copilotController.getYButton()) {
			m_backClimber.lowerClimber();
		}
		else {
			m_backClimber.setClimber(0.0);
		}
		//System.out.println("Elevator Encoder: \t" + m_elevator.getPosition());
		// Elevator move to position methods
		// m_elevator.moveToPosition(m_pilotController.getXButton() , State.HATCH_L1);
		// m_elevator.moveToPosition(m_pilotController.getYButton() , State.HATCH_L2);
		// m_elevator.moveToPosition(m_pilotController.getBumper(Hand.kLeft) , State.HATCH_L3);
		// m_elevator.moveToPosition(m_pilotController.getBumper(Hand.kRight), State.LEVEL_ZERO);

		// Hatch Mech

	}
}
