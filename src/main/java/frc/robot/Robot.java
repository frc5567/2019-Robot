/**
 * The main class that defines the actions and characteristics of our robot.
 */

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Robot extends TimedRobot {
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

 
  @Override
  public void robotPeriodic() {
  }

 
  @Override
  public void autonomousInit() {
   
  }


  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {

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
  }
}
