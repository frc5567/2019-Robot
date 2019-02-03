/**
 * This class defines the mechanism that allows the robot to move backwards and forwards using motors and wheels. 
 */

package frc.robot;

import frc.robot.RobotMap;

// Imports needed for motor controllers, speedc controller groups, and the drivetrain
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import edu.wpi.first.wpilibj.SpeedControllerGroup;
// These imports are extending SpeedController, allowing us to use SpeedControllerGroup
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Drivetrain {

    // Declares variables for drivetrain speed and rotate setter
    // Constant is how large each step is in the setters
    private final double MAX_DELTA_SPEED = 0.1;
    private final double MAX_QUICK_TURN_SPEED = 0.1;

    // Declares variables for current speed and rotate rate
    // Variables used for feedback in speed setter and rotate setter
    double m_currentSpeed;
    double m_currentRotate;

    // Declares variables to enable quick turn when speed is low enough
    boolean m_quickTurnEnabled;

    // Declarations for the motor controllers
    private WPI_VictorSPX m_frontLeftMotor;
    private WPI_VictorSPX m_frontRightMotor;
    private WPI_TalonSRX m_backLeftMotor;
    private WPI_TalonSRX m_backRightMotor;

    // Declarations for the speed controller groups
    private SpeedControllerGroup m_leftMotors;
    private SpeedControllerGroup m_rightMotors;

    // Declaration for the drivetrain
    private DifferentialDrive m_drivetrain;


    /**
     * Constructor for the motor controller declarations and the drivetrain object
     */
    Drivetrain() {
        
        // Initializes the motorControllers using the ports passed in
        m_frontLeftMotor = RobotMap.m_frontLeftDriveMotor;
        m_frontRightMotor = RobotMap.m_frontRightDriveMotor;
        m_backLeftMotor = RobotMap.m_backLeftDriveMotor;
        m_backRightMotor = RobotMap.m_backRightDriveMotor;
        
        // Initializes the motor controller groups (left side motors and right side motors)
        m_leftMotors = new SpeedControllerGroup(m_frontLeftMotor, m_backLeftMotor);
        m_rightMotors = new SpeedControllerGroup(m_frontRightMotor, m_backRightMotor);
        
        // Initializes the drivetrain with the motor controller groups
        m_drivetrain = new DifferentialDrive(m_leftMotors, m_rightMotors);

        // Initializes feedback variables for speed setter and rotate setter
        // Setters use variables as feedback in order to "ramp" the output gradually
        m_currentSpeed = 0;
        m_currentRotate = 0;
    }


    /**
     *      || Currently in place if access is needed to DifferentialDrive methods ||
     *      || Might be used for initSendable or other methods within class ||
     * Gets the drivetrain object to use DifferentialDrive methods
     * @return The drivetrain object (DifferentialDrive)
     */
    public DifferentialDrive getDrivetrain() {
        return m_drivetrain;
    }

    /**
     * Sets the drivetrain motor to desired settings,
     * Acceleration limiter is implemented to prevent current spikes
     * from putting robot in brownout condition
     * @param desiredSpeed The desired robot speed along x-axis [-1.0..1.0] forward is positive
     * @param desiredRotate The desired robot turning speed along z-axis [-1.0..1.0] clockwise is positive
     */
    public void curvatureDrive(double desiredSpeed, double desiredRotate) {

        // If desired speed is higher than current speed by a margin larger than kMaxDeltaSpeed,
        // Increase current speed by kMaxDelaSpeed's amount
        if (desiredSpeed > (m_currentSpeed + MAX_DELTA_SPEED)) {
            m_currentSpeed += MAX_DELTA_SPEED;
        }
        // If desired speed is less than current speed by a margin larger than kMaxDeltaSpeed
        // Decrease current speed by kMaxDeltaSpeed's amount
        else if (desiredSpeed < (m_currentSpeed - MAX_DELTA_SPEED)) {
            m_currentSpeed -= MAX_DELTA_SPEED;
        }
        // If desired speed is within kMaxDeltaSpeed's margin to current speed,
        // Set current speed to match desired speed
        else {
            m_currentSpeed = desiredSpeed;
        }

        // If desired rotate is higher than current rotate by a margin larger than kMaxDeltaSpeed,
        // Increase current rotate by kMaxDelaSpeed's amount
        if (desiredRotate > (m_currentRotate + MAX_DELTA_SPEED)) {
            m_currentRotate += MAX_DELTA_SPEED;
        }
        // If desired rotate is less than current rotate by a margin larger than kMaxDeltaSpeed
        // Decrease current rotate by kMaxDeltaSpeed's amount
        else if (desiredRotate < (m_currentRotate - MAX_DELTA_SPEED)) {
            m_currentRotate -= MAX_DELTA_SPEED;
        }
        // If desired rotate is within kMaxDeltaSpeed's margin to current rotate,
        // Set current rotate to match desired speed
        else {
            m_currentRotate = desiredRotate;
        }

        if ((m_currentSpeed < MAX_QUICK_TURN_SPEED) && (m_currentSpeed > MAX_QUICK_TURN_SPEED)) {
            m_quickTurnEnabled = true;
        }
        else {
            m_quickTurnEnabled = false;
        }

        // Pass current speed, current rotate, and the quick turn boolean into the Differential Drive's curatureDrive method
        m_drivetrain.curvatureDrive(m_currentSpeed, m_currentRotate, m_quickTurnEnabled);
    }
}
