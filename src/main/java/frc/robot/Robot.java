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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

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
	AutoCommands m_autoCommands;
	// Declare Teleop commands for pilot controller methods
	TeleopCommands m_teleopCommands;

	// Declare Dashboard and Dashboard data bus
	DashboardData m_dataStream;
	CustomDashboard m_roboDash;

	// Declaring the USB Camera
	UsbCamera camera;
	HatchMech m_hatchMech;

	WPI_VictorSPX liftDriveMotor;

	Robot() {

		// Instanciates drivetrain, driver controllers, climbers, and elevator
		m_pilotController = new Controller(RobotMap.PILOT_CONTROLLER_PORT);

		m_elevator = new Elevator();
		
		m_copilotController = new Controller(RobotMap.COPILOT_CONTROLLER_PORT);
		m_copilotGamepad = new GamePad(RobotMap.COPILOT_CONTROLLER_PORT);

		// Instantiates the front and back climbers with their respective motor and break beam ports
		m_frontClimber = new Climber(RobotMap.FRONT_CLIMBER_MOTOR_PORT, RobotMap.FRONT_CLIMBER_LIMIT_TOP_PORT, RobotMap.FRONT_CLIMBER_LIMIT_BOTTOM_PORT);
		m_backClimber = new Climber(RobotMap.BACK_CLIMBER_MOTOR_PORT, RobotMap.BACK_CLIMBER_LIMIT_TOP_PORT, RobotMap.BACK_CLIMBER_LIMIT_BOTTOM_PORT);
		
		// Instantiates elevator
		m_elevator = new Elevator();
		
		// Calls method to configure the PID settings for the elevator
		m_elevator.elevatorPIDConfig();
		
		// Instantiates hatch arm class 
		m_hatchMech = new HatchMech();

		liftDriveMotor = new WPI_VictorSPX(16);
		
		try {
			m_ahrs = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
		}

		m_drivetrain = new Drivetrain(m_ahrs);

		// This requires the arduino to be plugged in, otherwise, it will fail
		try {
			m_pather = new Pathing(m_drivetrain, m_ahrs);
		} catch (Exception e) {
			System.out.println("Pather failed to instantiate");
		}

		// Runs config for the PID system on the drivetrain
		m_drivetrain.talonDriveConfig();

		m_autoCommands = new AutoCommands(m_drivetrain, m_ahrs, m_elevator, m_frontClimber, m_backClimber);
		m_teleopCommands = new TeleopCommands(m_pilotController, m_copilotGamepad, m_drivetrain, m_elevator, m_frontClimber, m_backClimber, m_hatchMech);
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Sets up the camera and inits the camera server
		// This needs the camera to be plugged in
		try {
			camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(160, 120);
			camera.setFPS(1);			
		} catch (Exception e) {
			System.out.println("Camera failed to instantiate");
		}
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
		// Resets flags on the pather
		if (m_pather != null) {
			m_pather.resetFlags();
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		m_teleopCommands.teleopModeCommands();

		// Test drivetrain included, uses Left stick Y for speed, Right stick X for
		// turning, quick turn is auto-enabled at low speed


		// PID based sample talon arcade drive
		m_drivetrain.talonArcadeDrive(m_pilotController.getRightTrigger() - m_pilotController.getLeftTrigger(), m_pilotController.getLeftStickX());

		if(m_pilotController.getYButton()) {
			if (m_pather != null) {
				m_pather.pathToTarget();
			}
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
		if (m_pilotController.getBButton()) {
			liftDriveMotor.set(0.4);
		}
		else {
			liftDriveMotor.set(0.0);
		}
		*/
		/*
		if (m_pilotController.getYButton()) {
			m_frontClimber.setClimber(-0.6);
			//m_frontClimber.raiseClimber();
			m_backClimber.raiseClimber();
		}
		// Lowers both climbers at once
		// Start button
		else if (m_pilotController.getAButton()) {
			m_frontClimber.setClimber(0.2);
			//m_frontClimber.lowerClimber();
			m_backClimber.lowerClimber();
		}
		else {
			m_frontClimber.setClimber(0.0);
			m_backClimber.setClimber(0.0);
		}
		*/
		m_drivetrain.talonArcadeDrive(m_pilotController.getRightTrigger() - m_pilotController.getRightTrigger(), m_pilotController.getLeftStickX());
		
		// Zeros yaw if 'A' is pressed, and adds 180 degree offset if 'B' is pressed
		// if (m_pilotController.getAButtonReleased()) {
		// m_ahrs.zeroYaw();
		// }
		// if (m_pilotController.getBButtonReleased()) {
		// m_ahrs.flipOffset();
		// }

		// Prints yaw and if offset is applied to console
		// System.out.println(m_ahrs.getOffsetYaw() + "\t\t" +
		// m_ahrs.getOffsetStatus());

		// [NOTE] Negative power moves the elevator up, but the encoder will still tic
		// positive. This is due to the way the string is wound on the winch
		// Follow up: This is no longer quite true. So long as we call the elevator PID
		// config, the motor will be inverted, thus positive should be up

		// On pilot controlller
		// Lowers Elevator
		// A button		
		if (m_pilotController.getAButton()) {
			m_elevator.moveRaw(-.4);
		}
		// Raises elevator
		// B button
		else if (m_pilotController.getBButton()) {
			m_elevator.moveRaw(.4);
		}
		// Sets elevator to hatch level 1 state
		//X button
		else if (m_pilotController.getXButton()) {
			m_elevator.elevatorPIDDrive(State.HATCH_L1);
			// m_elevator.moveToPosition(m_pilotController.getXButton() , State.HATCH_L1);
		}
		// Sets elevator to hatch level 2 state
		// Y button
		else if (m_pilotController.getYButton()) {
			m_elevator.elevatorPIDDrive(State.HATCH_L2);
			// m_elevator.moveToPosition(m_pilotController.getYButton() , State.HATCH_L2);
		}
		// Sets elevator to hatch level 3 state
		// Right bumper
		else if (m_pilotController.getBumper(Hand.kRight)) {
			m_elevator.elevatorPIDDrive(State.HATCH_L3);
			// m_elevator.moveToPosition(m_pilotController.getBumper(Hand.kRight) ,
			// State.HATCH_L3);
		}
		// Sets elevator to level 0 state (starting position / bottom)
		// Left bumper
		else if (m_pilotController.getBumper(Hand.kLeft)) {
			m_elevator.elevatorPIDDrive(State.LEVEL_ZERO);
		}
		// Sets elevator speed to 0
		// No buttons
		else {
			m_elevator.moveRaw(0);
		}
		
		// Hatch arm controller bound to the copilot controller
		// Raise the arm on Y button
		// Lower the arm on X button
		if (m_copilotController.getYButton()) {
			m_hatchMech.armUp();
		}
		else if (m_copilotController.getXButton()) {
			 m_hatchMech.armDown();
		}
		else {
			m_hatchMech.setArm(0.0);
		}

		//*/

		// On copilot controller
		/*
		// Raises both climbers at once
		// Back button
		if (m_copilotController.getBackButton()) {
			m_frontClimber.raiseClimber();
			m_backClimber.raiseClimber();
		}
		// Lowers both climbers at once
		// Start button
		else if (m_copilotController.getStartButton()) {
			m_frontClimber.lowerClimber();
			m_backClimber.lowerClimber();
		}
		// Otherwise takes commands for seperate control
		else {
		*/
		/*
			// Raises the front climber
			// A button
			if (m_copilotController.getAButton()) {
				m_frontClimber.setClimber(-0.4);
			}
			// Lowers front climber
			// B button
			else if (m_copilotController.getBButton()) {
				m_frontClimber.setClimber(1.0);
			}
			// Sets front climber speed to 0
			// No buttons
			else {
				m_frontClimber.setClimber(0.0);
			}

			// Raises back climber
			// X button
			if (m_copilotController.getXButton()) {
				m_backClimber.setClimber(-0.4);
			}
			// Lowers back climber
			// Y button
			else if (m_copilotController.getYButton()) {
				m_backClimber.setClimber(0.9);
			}
			// Sets back climber speed to 0
			// Not buttons
			else {
				m_backClimber.setClimber(0.0);
			}
		/*
		}
		//*/
		
		// Arm servo controls bound to copilot controller
		// On A button released, open
		// On B button released, close
		if (m_copilotController.getAButtonReleased()){
			m_hatchMech.openServo();
		}
		else if(m_copilotController.getBButtonReleased()){
			m_hatchMech.closeServo();
		}
		
		System.out.print("Left Ultrasonics: \t" + m_drivetrain.getLeftUltra().getRangeInches());
		System.out.print("Right Ultrasonics: \t" + m_drivetrain.getRightUltra().getRangeInches());
		System.out.print("Drivetrain Enc Velocity: \t" + m_drivetrain.getLeftDriveEncoderVelocity() + "\t\t" + m_drivetrain.getRightDriveEncoderVelocity());
		System.out.print("Drivetrain Enc Pos: \t"+ m_drivetrain.getLeftDriveEncoderPosition() + "\t\t" + m_drivetrain.getRightDriveEncoderPosition());	
		System.out.print("Elevator Enc Velocity: \t" + m_elevator.m_elevatorMotor.getSelectedSensorVelocity());
		System.out.print("Elevator Enc Pos: \t"+ m_elevator.m_elevatorMotor.getSelectedSensorPosition());
		System.out.print("Front Break Beams: \t  Top: " + m_frontClimber.getTopLimitSwitch() + "\t Bottom: " + m_frontClimber.getBottomLimitSwitch());
		System.out.println("Back Break Beams: \t  Top: " + m_backClimber.getTopLimitSwitch() + "\t Bottom: " + m_backClimber.getBottomLimitSwitch());
		
	}
}
