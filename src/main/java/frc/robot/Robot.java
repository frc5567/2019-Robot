/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

import frc.robot.Drivetrain;
import frc.robot.Controller;
import frc.robot.Climber;
import frc.robot.NavX;
import frc.robot.Elevator;

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

	// Declare Pilot XBox Controller
	Controller m_pilotController;

	// Declare climbing mechanisms for front and back climbers
//	Climber m_frontClimber;
//	Climber m_backClimber;

	// Declare NavX
	NavX m_ahrs;

	// Declares Elevator
//	Elevator m_elevator;

	// Declare Auto Commands class for auto and auto assist commands
	AutoCommands autoCommands;

	// Declaring the USB Camera
//	UsbCamera camera;

	Robot() {

		m_drivetrain = new Drivetrain();
		m_pilotController = new Controller(RobotMap.PILOT_CONTROLLER_PORT);
//		m_frontClimber = new Climber(RobotMap.FRONT_CLIMBER_MOTOR_PORT, RobotMap.FRONT_CLIMBER_LIMIT_TOP_PORT);
//		m_backClimber = new Climber(RobotMap.BACK_CLIMBER_MOTOR_PORT, RobotMap.BACK_CLIMBER_LIMIT_TOP_PORT);
//		m_elevator = new Elevator();

		// Instantiate our duino to rio communication port
		// m_duinoToRio = new DuinoToRioComms();

		try {
			m_ahrs = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
		}

//		autoCommands = new AutoCommands(m_drivetrain, m_ahrs, m_elevator, m_frontClimber, m_backClimber);
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
			m_drivetrain.driveToPosition(36);
			System.out.println("Why are buttons?");
		}
		else {
			m_drivetrain.curvatureDrive(m_pilotController.getLeftStickY(), m_pilotController.getRightStickX());
		}
		
		if (m_pilotController.getAButtonReleased()) {
			m_ahrs.zeroYaw();
		}
		if (m_pilotController.getBButtonReleased()) {
			m_ahrs.flipOffset();
		}

//		System.out.println(m_ahrs.getOffsetYaw() + "\t\t" + m_ahrs.getOffsetStatus());
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
		
	}
}
