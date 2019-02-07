/**
 * The main class that defines the actions and characteristics of our robot.
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

=======
=======
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Drivetrain;
import frc.robot.Controller;
import frc.robot.RobotMap;
<<<<<<< HEAD
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
=======
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
=======

import frc.robot.Drivetrain;
import frc.robot.Controller;
import frc.robot.Climber;
>>>>>>> 2dbba8b1dddb1c9cc3c69bbd6fc18f6de6566bf1

public class Robot extends TimedRobot {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
  // This declares the elevator, pilot drive, the left and right stick vales, the front left and right motors, the back left and right motors; and the bolean quick rotate
  
  //User defined mechanisms
  Elevator m_elevator;

  XboxController m_pilotDrive;
  DifferentialDrive m_myDrive;

  //Defines left and right joystick values on pilot controller   
  double m_pilotLeftYStickValue; 
  double m_pilotRightXStickValue;
  boolean m_quickRotate;                // When enabled allows the robot to rotate in place  

  
  // Declares drive train motors
  SpeedController m_frontLeftDriveMotor;
  SpeedController m_backLeftDriveMotor;
  SpeedController m_frontRightDriveMotor;
  SpeedController m_backRightDriveMotor;

  //making the moters into two groups, right and left
  
  Robot() {
    m_pilotDrive = new XboxController(0);

    //Puts the left and right motors into two groups 
    SpeedControllerGroup m_leftSide = new SpeedControllerGroup (m_frontLeftDriveMotor, m_backLeftDriveMotor);
    SpeedControllerGroup m_rightSide =new SpeedControllerGroup (m_frontRightDriveMotor, m_backRightDriveMotor);
    DifferentialDrive m_myDrive = new DifferentialDrive (m_leftSide, m_rightSide);

  }

  @Override
  public void robotInit() {
  
  }

 
=======
=======
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0


  Drivetrain m_drivetrain;
  Controller m_pilotController;

  Robot() {

    // Declare our duino communication port
    DuinoToRioComms duinoToRio;
    
    m_drivetrain  = new Drivetrain();
    m_pilotController = new Controller(RobotMap.PILOT_CONTROLLER_PORT);

    // Instantiate our duino to rio communication port
    duinoToRio = new DuinoToRioComms();
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
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
  @Override
  public void robotPeriodic() {
  }

<<<<<<< HEAD
 
  @Override
  public void autonomousInit() {
   
=======
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
    
<<<<<<< HEAD
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
=======
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
  }


  @Override
  public void autonomousPeriodic() {
    
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
  }

  /**
   * This function is called once before the operator control period starts
   */
  @Override
  public void teleopInit() {
    
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
  }

  /**
   * This function is called once before the operator control period starts
   */
  @Override
  public void teleopInit() {
    
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
  }

  @Override
<<<<<<< HEAD
  public void teleopInit() {

=======
  public void teleopPeriodic() {
    // if (pilotController.getAButtonReleased()) {
    //   duinoToRio.pixyRead(2);
    // }
    // else if (pilotController.getBButtonReleased()) {
    //   duinoToRio.pixyRead(1);
    // }
    // Test drivetrain included, uses Left stick Y for speed, Right stick X for turning, and A button is held for quickturn
    m_drivetrain.curvatureDrive(m_pilotController.getLeftStickY(), m_pilotController.getRighStickX());
<<<<<<< HEAD
=======
  }

  /**
   * This function is called once before starting test mdoe
   */
  @Override
  public void testInit() {
    
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
  }

  /**
   * This function is called once before starting test mdoe
=======
  //  Test doubles for storing return from read classes
  Double degToTarget = Double.NaN;
  Double distToTarget = Double.NaN;

	// Declare drivetrain 
	Drivetrain m_drivetrain;
	Controller m_pilotController;
	Climber m_frontClimber;
  Climber m_backClimber;
  
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
	}

	/**
	 * This function is called once before starting test mdoe
	 */
	@Override
	public void testInit() {

	}

	/**
   * This function is called periodically during test mode.
>>>>>>> 2dbba8b1dddb1c9cc3c69bbd6fc18f6de6566bf1
   */
  @Override
  public void testInit() {
    
>>>>>>> c3abefed1d62e220e5bb85953dd300f5017b8fe0
  }

  @Override
  public void teleopPeriodic() {
    /**
     * TODO:
     * Move deadbands into seperate controller class
     * Look into built in deadbands
     */
    //This makes the deadbands. Pilot control is our variable for user input

    // This checks the stick Y value to see if it is less than 0.05 and greater than -0.05
    // If it is, it sets the left stick Y value to 0. If not, it sets the left stick Y value to the left stick Y value that is gotten from the controller
    if((m_pilotDrive.getY(Hand.kLeft)<0.05) && (m_pilotDrive.getY(Hand.kLeft)>-0.05)){
      m_pilotLeftYStickValue = 0;
      
    } else {
      m_pilotLeftYStickValue = m_pilotDrive.getY(Hand.kLeft);
    }

    // This checks to see if the stick X value is less than 0.05 and greater than -0.05. 
    //If it is, it sets the right stick X value to 0. If not, it sets the right stick X value to the right stick X value that is gotten from the controller
    if((m_pilotDrive.getX(Hand.kRight)<0.05) && (m_pilotDrive.getX(Hand.kRight)>-0.05)){
      m_pilotRightXStickValue = 0;
      
    } else {
      m_pilotRightXStickValue = m_pilotDrive.getX(Hand.kRight);
    }    

    /**
     * TODO:
     * Move into a seperate controller class
     */
    //Checks to see if the A button is being pushed. If it is it enables the quick rotate, if not it doesn't enable it.
    if(m_pilotDrive.getAButton()){
      m_quickRotate = true;
    }
    else {
      m_quickRotate = false;
    }

    //We'll eventually need to invert one of the controls
    m_myDrive.curvatureDrive(m_pilotLeftYStickValue, m_pilotRightXStickValue, m_quickRotate);

    
  }
  //Quin should get more information on Talon->Victor pairs for 
  //the final motor controllers
 
  @Override
  public void testPeriodic() {
    //  Code for testing comms with arduino
    if (m_pilotController.getAButtonReleased()) {
      //  Assigns return value. Checking NaN should occur here
      degToTarget = m_duinoToRio.getDegToTarget();
      if (distToTarget.isNaN()){
        System.out.println("No number returned");
      }
      else {
        System.out.println("degToTarget: " + degToTarget);
        m_pkt.degTargetHigh = degToTarget;
      }

    }
    else if (m_pilotController.getBButtonReleased()) {
      //  Assigns return value. Checking NaN should occur here
      distToTarget = m_duinoToRio.getDistToTarget();
      if (distToTarget.isNaN()){
        System.out.println("No number returned");
      }
      else {
        System.out.println("distToTarget: " + distToTarget);
        m_pkt.distTargetHigh = distToTarget;
      }

    }
    
  }
}
