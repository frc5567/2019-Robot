/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import frc.robot.Drivetrain;
import frc.robot.Controller;
import frc.robot.Climber;
import frc.robot.NavX;
import frc.robot.Elevator.State;
import frc.robot.Elevator;
import frc.robot.HatchMech;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	// Test doubles for storing return from read classes
	private Double m_degToTarget = Double.NaN;
	private Double m_distToTarget = Double.NaN;
	private Double m_angleToCenter = Double.NaN;
	private Double m_lowPosition = Double.NaN;

	// Declare drivetrain
	Drivetrain m_drivetrain;

	// Declare Pilot XBox Controller
	Controller m_pilotController;

	// Declares xbox controller for co-pilot
	// Used for testing, gamepad will be used in comp
	Controller m_copilotController;

	// Declare climbing mechanisms for front and back climbers
	Climber m_frontClimber;
	Climber m_backClimber;

	// Declare NavX
	NavX m_ahrs;

	// Declares Elevator
	Elevator m_elevator;

	// Declare Auto Commands class for auto and auto assist commands
	AutoCommands autoCommands;

	// Declare Dashboard and Dashboard data bus
	DashboardData m_dataStream;
	CustomDashboard m_roboDash;

	// Declares the hatchmech class for the hatch arm and servo
	HatchMech m_hatchMech;

	WPI_VictorSPX liftDriveMotor;

	// Declare our duino communication port
	// private DuinoToRioComms m_duinoToRio;
	// private DuinoCommStorage m_pkt;

	Robot() {

		// Instantiates drivetrain
		m_drivetrain = new Drivetrain();
		// Instantiates pilot and copilot xbox controllers with their respective ports
		m_pilotController = new Controller(RobotMap.PILOT_CONTROLLER_PORT);
		m_copilotController = new Controller(RobotMap.COPILOT_CONTROLLER_PORT);
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

		// Instantiate our duino to rio communication port
		// m_duinoToRio = new DuinoToRioComms();

		try {
			/*
			 * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART) --
			 * 
			 * and USB. - See
			 * 
			 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and --
			 * 
			 * USB. - See
			 * 
			 * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
			 * 
			 * Multiple navX-model devices on a single robot are supported. //
			 ************************************************************************/
			m_ahrs = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
		}

		// Instanciates auto commands class for using auto assist
		autoCommands = new AutoCommands(m_drivetrain, m_ahrs, m_elevator, m_frontClimber, m_backClimber);
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

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
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {

		// Test drivetrain included, uses Left stick Y for speed, Right stick X for
		// turning, quick turn is auto-enabled at low speed
		m_drivetrain.curvatureDrive(m_pilotController.getLeftStickY(), m_pilotController.getRightStickX());

		// Zeros yaw if 'A' is pressed, and adds 180 degree offset if 'B' is pressed
		if (m_pilotController.getAButtonReleased()) {
			m_ahrs.zeroYaw();
		}
		if (m_pilotController.getBButtonReleased()) {
			m_ahrs.flipOffset();
		}

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
		// m_drivetrain.curvatureDrive(m_pilotController.getLeftStickY(), m_pilotController.getLeftStickX());
		m_drivetrain.getDrivetrain().arcadeDrive(m_pilotController.getLeftStickY(), m_pilotController.getLeftStickX());
		System.out.println("Drivetrain Enc Velocity \t" + m_drivetrain.getLeftDriveEncoderVelocity() + "\t\t" + m_drivetrain.getRightDriveEncoderVelocity());
		System.out.println("Drivetrain Enc Pos \t"+ m_drivetrain.getLeftDriveEncoderPosition() + "\t\t" + m_drivetrain.getRightDriveEncoderPosition());	
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
		// m_drivetrain.curvatureDrive(m_pilotController.getLeftStickY(),
		// m_pilotController.getRightStickX());

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

		// New Stuff
		// Elevator controls, triggers are for testing as of 2/16
		// System.out.println(m_pilotController.getLeftTrigger() -
		// m_pilotController.getRightTrigger());
		// m_elevator.moveRaw(m_pilotController.getLeftTrigger() -
		// m_pilotController.getRightTrigger());

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
		
		//*/
		//*/
		// Resets elevator encoder to 0
		// Start button
		/*
		if (m_pilotController.getStartButton()) {
			m_elevator.m_elevatorEncoder.setQuadraturePosition(0, 0);
		}
		*/

		
		// Copilot controller
		// Raises the hatch arm up
		// Left stick button
		if (m_copilotController.getYButton()) {
			m_hatchMech.ArmUp();
		}
		// Lowers the hatch arm down
		// Right stick button
		else if (m_copilotController.getXButton()) {
			 m_hatchMech.ArmDown();
		}
		// Sets the hatch arm speed to 0
		// No buttons
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
		
		if (m_copilotController.getAButtonReleased()){
			m_hatchMech.OpenServo();
		}
		else if(m_copilotController.getBButtonReleased()){
			m_hatchMech.CloseServo();
		}
		
		// System.out.println("Elevator Encoder: \t" + m_elevator.getPosition());
		// Elevator move to position methods
		// m_elevator.moveToPosition(m_pilotController.getXButton() , State.HATCH_L1);
		// m_elevator.moveToPosition(m_pilotController.getYButton() , State.HATCH_L2);
		// m_elevator.moveToPosition(m_pilotController.getBumper(Hand.kLeft) ,
		// State.HATCH_L3);
		// m_elevator.moveToPosition(m_pilotController.getBumper(Hand.kRight),
		// State.LEVEL_ZERO);

		// Hatch Mech

	}
}
