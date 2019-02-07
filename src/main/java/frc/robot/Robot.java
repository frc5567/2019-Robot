/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.Drivetrain;
import frc.robot.Controller;
import frc.robot.Climber;
import frc.robot.NavX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
	// Test doubles for storing return from read classes
	Double degToTarget = Double.NaN;
	Double distToTarget = Double.NaN;

	// Declare drivetrain
	Drivetrain m_drivetrain;
	Controller m_pilotController;
	Climber m_frontClimber;
	Climber m_backClimber;

	// Declare NavX
	NavX ahrs;

	// Declare our duino communication port
	private DuinoToRioComms m_duinoToRio;
	private DuinoCommStorage m_pkt;

	Robot() {

		m_drivetrain = new Drivetrain();
		m_pilotController = new Controller(RobotMap.PILOT_CONTROLLER_PORT);
		m_frontClimber = new Climber(RobotMap.FRONT_CLIMBER_MOTOR_PORT, RobotMap.FRONT_CLIMBER_LIMIT_TOP_PORT);
		m_backClimber = new Climber(RobotMap.BACK_CLIMBER_MOTOR_PORT, RobotMap.BACK_CLIMBER_LIMIT_TOP_PORT);

		// Instantiate our duino to rio communication port
		m_duinoToRio = new DuinoToRioComms();

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
			ahrs = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
		}
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

		if (m_pilotController.getAButtonReleased()) {
			ahrs.zeroYaw();
		}
		if (m_pilotController.getBButtonReleased()) {
			ahrs.flipOffset();
		}
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
		// Code for testing comms with arduino
		if (m_pilotController.getAButtonReleased()) {
			// Assigns return value. Checking NaN should occur here
			degToTarget = m_duinoToRio.getDegToTarget();
			if (distToTarget.isNaN()) {
				System.out.println("No number returned");
			} else {
				System.out.println("degToTarget: " + degToTarget);
				m_pkt.degTargetHigh = degToTarget;
			}

		} else if (m_pilotController.getBButtonReleased()) {
			// Assigns return value. Checking NaN should occur here
			distToTarget = m_duinoToRio.getDistToTarget();
			if (distToTarget.isNaN()) {
				System.out.println("No number returned");
			} else {
				System.out.println("distToTarget: " + distToTarget);
				m_pkt.distTargetHigh = distToTarget;
			}

		}

	}
}
