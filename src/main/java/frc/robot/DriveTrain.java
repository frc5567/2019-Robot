/**
 * This class defines the mechanism that allows the robot to move backwards and forwards using motors and wheels. 
 */

package frc.robot;

//we imported the methods for the controller, differential drive, and VictorSP
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;


public class DriveTrain {

    private double desiredSpeed;
    private final double kMaxDeltaSpeed = 0.1;
    private double currentSpeed;

    private double m_motorPowerLeft= 0;
    private double m_motorPowerRight= 0;

    private XboxController m_driveController;

    private VictorSP m_frontLeftMotor;
    private VictorSP m_frontRightMotor;
    private VictorSP m_backLeftMotor;
    private VictorSP m_backRightMotor;

    private SpeedControllerGroup m_leftMotors;
    private SpeedControllerGroup m_rightMotors;

    private DifferentialDrive m_drivetrain;


    public void initializeDrivetrain(int frontLeftChannel, int frontRightChannel, int backLeftChannel, int backRightChannel) {
        m_frontLeftMotor = new VictorSP(frontLeftChannel);
        m_frontRightMotor = new VictorSP(frontRightChannel);
        m_backLeftMotor = new VictorSP(backLeftChannel);
        m_backRightMotor = new VictorSP(backRightChannel);

        m_leftMotors = new SpeedControllerGroup(m_frontLeftMotor, m_backLeftMotor);
        m_rightMotors = new SpeedControllerGroup(m_frontRightMotor, m_backRightMotor);

        m_drivetrain = new DifferentialDrive(m_leftMotors, m_rightMotors);
    }
}