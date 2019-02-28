/**
 * This class defines the mechanism that allows the robot to move backwards and forwards using motors and wheels. 
 */

package frc.robot;

//we imported the methods for the controller, differential drive, and VictorSP
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import edu.wpi.first.wpilibj.VictorSP;


public class DriveTrain {
    private double m_motorPowerLeft= 0;
    private double m_motorPowerRight= 0;
    private XboxController m_driveController;
    private VictorSP m_leftMotor;
    private VictorSP m_rightMotor;
    private DifferentialDrive m_driveTrain;

}