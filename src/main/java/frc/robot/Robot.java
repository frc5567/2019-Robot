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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Drivetrain;
import frc.robot.Controller;
import frc.robot.Climber;
import frc.robot.NavX;
import frc.robot.Elevator;
import frc.robot.HatchMech;
import frc.robot.AutoCommands;
import frc.robot.TeleopCommands;
import frc.robot.Elevator.State;
import frc.robot.GamePad;
import frc.robot.DriveClimber;
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

	// Declare pather for vision targeting
	Pathing m_pather;

	// Declare Pilot XBox Controller
	Controller m_controller;

	// Declare test controller, not for comp
	Controller m_testController;

	// Declare Copilot Gamepad
	GamePad m_gamepad;

	// Declare climbing mechanisms for front and back climbers
	Climber m_frontClimber;
	DriveClimber m_backClimber;

	// Declare NavX
	NavX m_gyro;

	// Declares Elevator
	Elevator m_elevator;

	// Declare Auto Commands class for auto and auto assist commands
	AutoCommands m_autoCommands;

	// Declare Teleop commands for pilot controller methods
	TeleopCommands m_teleopCommands;

	// Declare Dashboard and Dashboard data bus
	DashboardData m_dataStream;
	CustomDashboard m_roboDash;
	
	// Sendable chooser for selecting auton on the dashboard
	SendableChooser<String> m_autoChooser;
	private String m_autoSelected;

	// Declaring the USB Camera
	UsbCamera camera;
	HatchMech m_hatchMech;

	// Declare the continuous command sequence
	ContinuousCommand testContinuousCommand;

	// Declare PID control for the elevator
	ClimberPIDControl climberPID;

	// Declare ringlights for control via PCM
	Solenoid innerRingLight;
	Solenoid outerRingLight;

	// Counter for telemetry so we don't print every cycle
	int telemetryCounter;

	// TODO: TEMP FLAGS REMOVE PLEASE
	// Flags for running through auton sequence, currently in test
	boolean backFlag = false;
	boolean firstRotFlag = false;
	boolean backFlagTwo = false;
	boolean secondRotFlag = false;
	boolean forwardFlag = false;
	boolean leaveRocket = false;
	boolean forwardFlag2 = false;
	boolean forwardFlag3 = false;
	boolean forwardFlag4 = false;
	boolean hatchApproach = false;

	int m_forwardCounter = 0;

	boolean sandstormControl = false;

	Robot() {

		// Instantiates drivetrain, driver controllers, climbers, and elevator
		m_controller = new Controller(RobotMap.PILOT_CONTROLLER_PORT);

		m_testController = new Controller(2);

		m_elevator = new Elevator();
		
		m_gamepad = new GamePad(RobotMap.COPILOT_CONTROLLER_PORT);

		// Instantiates the front and back climbers with their respective motor and break beam ports
		m_frontClimber = new Climber(RobotMap.FRONT_CLIMBER_MOTOR_PORT, RobotMap.FRONT_CLIMBER_LIMIT_TOP_PORT, RobotMap.FRONT_CLIMBER_LIMIT_BOTTOM_PORT);
		m_backClimber = new DriveClimber(RobotMap.BACK_CLIMBER_MOTOR_PORT, RobotMap.BACK_CLIMBER_LIMIT_TOP_PORT, RobotMap.BACK_CLIMBER_LIMIT_BOTTOM_PORT, RobotMap.CLIMBER_DRIVE_MOTOR_PORT);
		
		// Instantiates elevator
		m_elevator = new Elevator();
		
		// Calls method to configure the PID settings for the elevator
		m_elevator.configPID();
		
		// Instantiates hatch arm class 
		m_hatchMech = new HatchMech();

		try {
			m_gyro = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
		}

		m_drivetrain = new Drivetrain(m_gyro);

		// This requires the arduino to be plugged in, otherwise, it will fail
		try {
			m_pather = new Pathing(m_drivetrain, m_gyro, m_controller);
		} catch (Exception e) {
			System.out.println("Pather failed to instantiate");
		}

		// Runs config for the PID system on the drivetrain
		m_drivetrain.talonDriveConfig();

		// Instantiates PID control
		climberPID = new ClimberPIDControl(m_frontClimber, m_backClimber, m_gyro);

		// Runs config for synced PID climbers
		climberPID.climberPIDConfig();

		innerRingLight = new Solenoid(RobotMap.PCM_PORT, RobotMap.INNER_RINGLIGHT_PORT);
		outerRingLight = new Solenoid(RobotMap.PCM_PORT, RobotMap.OUTER_RINGLIGHT_PORT);

		m_autoCommands = new AutoCommands(m_drivetrain, m_gyro, m_elevator, m_frontClimber, m_backClimber, m_pather, m_teleopCommands, m_hatchMech, outerRingLight, innerRingLight);
		m_teleopCommands = new TeleopCommands(m_controller, m_gamepad, m_drivetrain, m_elevator, m_frontClimber, m_backClimber, m_hatchMech, climberPID, m_pather, m_autoCommands);

		// Instantiates the SendableChooser used for setting auton mode
		m_autoChooser = new SendableChooser<>();

		// Sets up the camera and inits the camera server
		// This needs the camera to be plugged in
		try {
			camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(160, 120);
			camera.setFPS(10);			
		} catch (Exception e) {
			System.out.println("Camera failed to instantiate");
		}
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_drivetrain.talonDriveConfig();
		m_hatchMech.openServo();
		climberPID.climberPIDConfig();

		// An attempt to force reset of position
		climberPID.climberPIDConfig();

		m_gyro.reset();
		m_gyro.zeroYaw();

		// Setting up SendableChooser options used in auton.
		m_autoChooser.setDefaultOption("Teleop / Manual", RobotMap.TELEOP);
		m_autoChooser.addOption("Right Auton", RobotMap.RIGHT_AUTO);
		m_autoChooser.addOption("Left Auton", RobotMap.LEFT_AUTO);

		// TODO: Put this on the LiveWindow in shuffleboard instead of the SmartDashboard
		SmartDashboard.putData("Auto Choices", m_autoChooser);
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
		// Resets flags on the pather
		if (m_pather != null) {
			m_pather.resetFlags();
		}

		// Getting the selected auton
		m_autoSelected = m_autoChooser.getSelected();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		if (m_gamepad.getLevelZero()){
			sandstormControl = true;
			innerRingLight.set(false);
			outerRingLight.set(false);
		}

		if (sandstormControl) {
			m_teleopCommands.teleopModeCommands();
			return;
		}

		else {
			// TODO: This auton is based on test. Until it is thourghly tested ,we should always default to TELEOP
			switch (m_autoSelected) {
				case RobotMap.TELEOP:
					m_teleopCommands.teleopModeCommands();
					break;
				case RobotMap.RIGHT_AUTO:
					// Right auton code goes here
					if (!backFlag) {
						backFlag = m_drivetrain.driveToPositionAngle(-48, 0, .5);
					}
					else if (!backFlagTwo) {
						backFlagTwo = m_drivetrain.driveToPositionAngle(-130, 25, .9);
						m_elevator.drivePID(State.HATCH_PICKUP);
						m_hatchMech.armDown();
						m_hatchMech.closeServo();
			
					}
					else if (!secondRotFlag) {
						secondRotFlag = m_drivetrain.rotateToAngle(-29);
						m_hatchMech.armDown();
						outerRingLight.set(true);
						innerRingLight.set(true);
					}
					else if (!forwardFlag) {
						m_hatchMech.openServo(); 
						m_hatchMech.setArm(0);
						forwardFlag = m_pather.driveToTarget(9);
					}
					else if (m_forwardCounter < 20) {
						m_drivetrain.talonArcadeDrive(.15, 0, false);
						m_forwardCounter++;
						outerRingLight.set(false);
						innerRingLight.set(false);
					}
					else if (!leaveRocket) {
						leaveRocket = m_drivetrain.driveToPositionAngle(-36, -29, .3);
					}
					else if (!forwardFlag2) {
						forwardFlag2 = m_drivetrain.driveToPositionAngle(130, 0, .9);
					}
					else if (!forwardFlag3) {
						forwardFlag3 = m_drivetrain.driveToPositionAngle(24, -20, .5);
						// This sysout is for testing what stage in auton we are in, commented out for bandwidth when not testing
						// System.out.println("Currently in ForwardFlag3 - angled");
					}
					else if (!forwardFlag4) {
						// This sysout is for testing what stage in auton we are in, commented out for bandwidth when not testing
						// System.out.println("Currently in ForwardFlag4 - straight");
						forwardFlag4 = m_drivetrain.driveToPositionAngle(30, 0, .75);
						m_elevator.drivePID(State.HATCH_PICKUP);
						m_autoCommands.resetFlags();
					}
					else if (!hatchApproach) {
						hatchApproach = m_autoCommands.autoPickupAssist();
					}
					else {
						m_pather.resetFlags();
						m_autoCommands.resetFlags();
						outerRingLight.set(false);
						innerRingLight.set(false);
						m_teleopCommands.teleopModeCommands();
					}
					break;
				case RobotMap.LEFT_AUTO:
					// Left auton code goes here
					if (!backFlag) {
						backFlag = m_drivetrain.driveToPositionAngle(-48, 0, .5);
					}
					else if (!backFlagTwo) {
						backFlagTwo = m_drivetrain.driveToPositionAngle(-147, -25, .9);
						m_elevator.drivePID(State.HATCH_PICKUP);
						m_hatchMech.armDown();
						m_hatchMech.closeServo();
			
					}
					else if (!secondRotFlag) {
						secondRotFlag = m_drivetrain.rotateToAngle(29);
						m_hatchMech.armDown();
						outerRingLight.set(true);
						innerRingLight.set(true);
					}
					else if (!forwardFlag) {
						m_hatchMech.setArm(0);
						m_hatchMech.openServo();
						forwardFlag = m_pather.driveToTarget(9);
					}
					else if (m_forwardCounter < 20) {
						m_drivetrain.talonArcadeDrive(.15, 0, false);
						m_forwardCounter++;
						outerRingLight.set(false);
						innerRingLight.set(false);
					}
					else if (!leaveRocket) {
						leaveRocket = m_drivetrain.driveToPositionAngle(-36, 29, .3);
						
					}
					else if (!forwardFlag2) {
						forwardFlag2 = m_drivetrain.driveToPositionAngle(130, 0, .9);
					}
					else if (!forwardFlag3) {
						forwardFlag3 = m_drivetrain.driveToPositionAngle(24, 20, .5);
					}
					else if (!forwardFlag4) {
						forwardFlag4 = m_drivetrain.driveToPositionAngle(30, 0, .75);
						m_elevator.drivePID(State.HATCH_PICKUP);
						m_autoCommands.resetFlags();
					}
					else if (!hatchApproach) {
						hatchApproach = m_autoCommands.autoPickupAssist();
					}
					else {
						m_autoCommands.resetFlags();
						outerRingLight.set(false);
						innerRingLight.set(false);
						m_teleopCommands.teleopModeCommands();
					}
					break;
			}
		}
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
		
		innerRingLight.set(false);
		outerRingLight.set(false);
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		m_teleopCommands.teleopModeCommands();
	}

	/**
	 * This function is called once before starting test mdoe
	 */
	@Override
	public void testInit() {
		telemetryCounter = 0;
		m_gyro.reset();
		m_gyro.zeroYaw();
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		
		// if(m_testController.getAButton()) {
		// 	m_drivetrain.rotateToAngle(30);
		// }
		// else if(m_testController.getBButton()) {
		// 	m_drivetrain.rotateToAngle(15);
		// }
		// else if (m_testController.getXButton()) {
		// 	m_drivetrain.rotateToAngle(5);
		// }
		// else if (m_testController.getYButton()) {
		// 	m_drivetrain.driveToPositionAngle(100, 20, 1);
		// } 
		// else {
			 m_teleopCommands.teleopModeCommands();
			//m_drivetrain.talonArcadeDrive(0, 0, false);
			//m_drivetrain.m_firstCall = true;
		// }
		// System.out.println("Current Heading \t" + m_gyro.getYaw() + "\t Target Heading \t" + m_drivetrain.m_rotController.getSetpoint());
		
		// innerRingLight.set(true);
		// outerRingLight.set(true);
		
		if ((telemetryCounter % RobotMap.SAMPLE_RATE) == 0) {

			// System.out.println("Pitch: \t" + m_gyro.getRoll());

			if (RobotMap.ULTRASONIC_TELEMETRY) {
				System.out.print("Left Ultrasonics: \t" + m_drivetrain.getLeftUltra().getRangeInches());
				System.out.println(" Right Ultrasonics: \t" + m_drivetrain.getRightUltra().getRangeInches());
			}
			
			if (RobotMap.DRIVETRAIN_TELEMETRY) {
				System.out.print("Gyro Yaw: \t" + m_gyro.getYaw());
				System.out.print(" Drivetrain Enc Velocity: \t" + m_drivetrain.getLeftDriveEncoderVelocity() + "\t\t"  + m_drivetrain.getRightDriveEncoderVelocity());
				// System.out.println(" Drivetrain Pos: \t"+ (6*RobotMap.PI) * (m_drivetrain.m_masterLeftMotor.getSelectedSensorPosition() / 4096) + "\t\t" + m_drivetrain.getRightDriveEncoderPosition());	
				System.out.println(" Enc Pos: \t\t" + m_drivetrain.m_masterLeftMotor.getSelectedSensorPosition() + "\t\t" +m_drivetrain.m_masterRightMotor.getSelectedSensorPosition());
			}

			if (RobotMap.ELEVATOR_TELEMETRY) {
				System.out.print("Elevator Enc Velocity: \t" + m_elevator.m_motor.getSelectedSensorVelocity());
				System.out.println(" Elevator Enc Pos: \t"+ m_elevator.m_motor.getSelectedSensorPosition());
			}

			if (RobotMap.CLIMBER_TELEMETRY) {
				System.out.print(" Front Climber Enc Velocity: \t" + m_frontClimber.m_climberMotor.getSelectedSensorVelocity()); //getSelectedSensorVelocity());
				System.out.print(" Front Climber Enc Pos: \t"+ m_frontClimber.m_climberMotor.getSelectedSensorPosition());
				System.out.print(" Back Climber Enc Velocity: \t" + m_backClimber.m_climberMotor.getSelectedSensorVelocity()); //getSelectedSensorVelocity());
				System.out.println(" Back Climber Enc Pos: \t"+ m_backClimber.m_climberMotor.getSelectedSensorPosition());
			}
		}
		telemetryCounter++;
		
	}
}
