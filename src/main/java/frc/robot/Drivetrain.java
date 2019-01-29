/**
 * This class defines the mechanism that allows the robot to move backwards and forwards using motors and wheels. 
 */

package frc.robot;

// Imports needed for motor controllers, speedc controller groups, and the drivetrain
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;


public class Drivetrain {

    // Declares variables for drivetrain speed and rotate setter
    // Constant is how large each step is in the setters
    private final double kMaxDeltaSpeed = 0.1;
    private double currentSpeed;
    private double currentRotate;

    // Declarations for the motor controllers
    private VictorSP m_frontLeftMotor;
    private VictorSP m_frontRightMotor;
    private VictorSP m_backLeftMotor;
    private VictorSP m_backRightMotor;

    // Declarations for the speed controller groups
    private SpeedControllerGroup m_leftMotors;
    private SpeedControllerGroup m_rightMotors;

    // Declaration for the drivetrain
    private DifferentialDrive m_drivetrain;


    /**
     * Constructor for the motor controller declarations and the drivetrain object
     * @param frontLeftChannel The channel of the front left motor controller
     * @param frontRightChannel The channel of the front right motor controller
     * @param backLeftChannel The channel of the back left motor controller
     * @param backRightChannel The channel of the back right motor controller
     */
    Drivetrain(int frontLeftChannel, int frontRightChannel, int backLeftChannel, int backRightChannel) {
        
        // Initializes the motorControllers using the ports passed in
        m_frontLeftMotor = new VictorSP(frontLeftChannel);
        m_frontRightMotor = new VictorSP(frontRightChannel);
        m_backLeftMotor = new VictorSP(backLeftChannel);
        m_backRightMotor = new VictorSP(backRightChannel);
        
        // Initializes the motor controller groups (left side motors and right side motors)
        m_leftMotors = new SpeedControllerGroup(m_frontLeftMotor, m_backLeftMotor);
        m_rightMotors = new SpeedControllerGroup(m_frontRightMotor, m_backRightMotor);
        
        // Initializes the drivetrain with the motor controller groups
        m_drivetrain = new DifferentialDrive(m_leftMotors, m_rightMotors);
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
     * @param quickTurn If true, overrides constant-curvature turning for turn-in-place maneuvering
     */
    public void curvatureDrive(double desiredSpeed, double desiredRotate, boolean quickTurn) {

        // If desired speed is higher than current speed by a margin larger than kMaxDeltaSpeed,
        // Increase current speed by kMaxDelaSpeed's amount
        if (desiredSpeed > (currentSpeed + kMaxDeltaSpeed)) {
            currentSpeed += kMaxDeltaSpeed;
        }
        // If desired speed is less than current speed by a margin larger than kMaxDeltaSpeed
        // Decrease current speed by kMaxDeltaSpeed's amount
        else if (desiredSpeed < (currentSpeed - kMaxDeltaSpeed)) {
            currentSpeed -= kMaxDeltaSpeed;
        }
        // If desired speed is within kMaxDeltaSpeed's margin to current speed,
        // Set current speed to match desired speed
        else {
            currentSpeed = desiredSpeed;
        }

        // If desired rotate is higher than current rotate by a margin larger than kMaxDeltaSpeed,
        // Increase current rotate by kMaxDelaSpeed's amount
        if (desiredRotate > (currentRotate + kMaxDeltaSpeed)) {
            currentRotate += kMaxDeltaSpeed;
        }
        // If desired rotate is less than current rotate by a margin larger than kMaxDeltaSpeed
        // Decrease current rotate by kMaxDeltaSpeed's amount
        else if (desiredRotate < (currentRotate - kMaxDeltaSpeed)) {
            currentRotate -= kMaxDeltaSpeed;
        }
        // If desired rotate is within kMaxDeltaSpeed's margin to current rotate,
        // Set current rotate to match desired speed
        else {
            currentRotate = desiredRotate;
        }

        // Pass current speed, current rotate, and the quick turn boolean into the Differential Drive's curatureDrive method
        m_drivetrain.curvatureDrive(currentSpeed, currentRotate, quickTurn);
    }
}
