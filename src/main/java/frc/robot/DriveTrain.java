/**
 * This class defines the mechanism that allows the robot to move backwards and forwards using motors and wheels. 
 */

package frc.robot;

//importing the types (speed control, drive, and spark)
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.VictorSP;

//we imported the methods for the differential drive


public class DriveTrain {
    // Declare variable, and set numeric values
    //We'll eventually make one of the Victors a Talon SRX and have it lead the other
    //maybe clarify that m_frontLeft is a Victor or Talon
    VictorSP m_frontLeft = new VictorSP(0);
    VictorSP m_backLeft = new VictorSP(1);
    SpeedControllerGroup m_left = new SpeedControllerGroup(m_frontLeft, m_backLeft);

    VictorSP m_frontRight = new VictorSP(2);
    VictorSP m_backRight = new VictorSP(3);
    SpeedControllerGroup m_right = new SpeedControllerGroup(m_frontRight, m_backRight);

    DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);
    

  

    //Declaring drivetrain Speed controllers
 
    //Declaring speed controller group


    //Assign variable values
 
    //Set left & right joystick values to motor power left & motor power right variables
 

    //Set drive train power to motor variables
*/
}