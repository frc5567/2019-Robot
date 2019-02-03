/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Drivetrain;
import frc.robot.Controller;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Declare our duino communication port
  DuinoToRioComms duinoToRio;

  Drivetrain drivetrain;
  Controller pilotController;
  Controller copilotController;

  Robot() { 
    drivetrain  = new Drivetrain(0, 1, 2, 3);
    pilotController = new Controller(0);
    copilotController = new Controller(1);

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
    // if (pilotController.getAButtonReleased()) {
    //   duinoToRio.pixyRead(2);
    // }
    // else if (pilotController.getBButtonReleased()) {
    //   duinoToRio.pixyRead(1);
    // }
    // Test drivetrain included, uses Left stick Y for speed, Right stick X for turning, and A button is held for quickturn
    drivetrain.curvatureDrive(pilotController.getLeftStickY(), pilotController.getRighStickX(), pilotController.getAButton());
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
