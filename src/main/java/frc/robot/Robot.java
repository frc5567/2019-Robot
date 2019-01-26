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

  Elevator m_elevator;
  XboxController pilotDrive;
    
  double value; 
  double stickValueLeft; 
  double stickValueRight;

  SpeedController frontLeftMotor;
  SpeedController backLeftMotor;
  SpeedController frontRightMotor;
  SpeedController backRightMotor;

  //making the moters into two groups, right and left
  SpeedControllerGroup leftSide = new SpeedControllerGroup (frontLeftMotor, backLeftMotor);
  SpeedControllerGroup RightSide =new SpeedControllerGroup (frontRightMotor, backRightMotor);
  DifferentialDrive myDrive = new DifferentialDrive (leftSide, RightSide);
  Robot() {
    pilotDrive = new XboxController(0);
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

    //This makes the deadbands, pilot control is our variable for user input
    if((pilotDrive.getY(Hand.kLeft)<0.05) && (pilotDrive.getY(Hand.kLeft)>-0.05)){
      stickValueLeft = 0;
      
    } else {
      stickValueLeft = pilotDrive.getY(Hand.kLeft);
    }

    if((pilotDrive.getY(Hand.kRight)<0.05) && (pilotDrive.getY(Hand.kRight)>-0.05)){
      stickValueRight = 0;
      
    } else {
      stickValueLeft = pilotDrive.getY(Hand.kRight);
    }    
    //We'll eventually need to inverone of the controls
   
    myDrive.tankDrive(stickValueLeft, stickValueRight);

    
  }
  //Quin should get more information on Talon->Victor pairs for 
  //the final motor controllers
 
  @Override
  public void testPeriodic() {
  }
}
