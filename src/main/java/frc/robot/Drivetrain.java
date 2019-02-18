package frc.robot;

import edu.wpi.first.wpilibj.PIDOutput;
// Imports needed for motor controllers, speed controller groups, and the drivetrain
import edu.wpi.first.wpilibj.drive.DifferentialDrive; 
// These imports are extending SpeedController, allowing us to use SpeedControllerGroup
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.SensorCollection;

// Import needed to initialize NavX and rotation controller
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class defines the main drivetrain used with CAN motor controllers
 * @author Matt, Josh
 * @version Week 5 Pre-comp
 */
public class Drivetrain implements PIDOutput{
    // Declares NavX for rotation control
    private NavX m_ahrs;

    // Declares turn control PID
    private PIDController m_rotController;

    // Delcares a flag for checking if this is the first time entering this method in a given run
    boolean m_firstCall = true;

    // Declares variables for current speed and rotate rate
    // Variables used for feedback in speed setter and rotate setter
    double m_currentSpeed;
    double m_currentRotate;

    // Declares variables to enable quick turn when speed is low enough
    boolean m_quickTurnEnabled;

    // Declarations for the motor controllers
    private WPI_VictorSPX m_slaveLeftMotor;
    private WPI_VictorSPX m_slaveRightMotor;
    private WPI_TalonSRX m_masterLeftMotor;
    private WPI_TalonSRX m_masterRightMotor;

    // Declaration for encoders connected to TalonSRXs
    private SensorCollection m_leftDriveEncoder;
    private SensorCollection m_rightDriveEncoder;

    // Declaration for the drivetrain
    private DifferentialDrive m_drivetrain;


    /**
     * Constructor for the motor controller declarations and the drivetrain object
     */
    public Drivetrain() {
        
        // Initializes the motorControllers using the ports passed in
        m_slaveLeftMotor = new WPI_VictorSPX(RobotMap.SLAVE_LEFT_DRIVE_MOTOR_PORT);
        m_slaveRightMotor = new WPI_VictorSPX(RobotMap.SLAVE_RIGHT_DRIVE_MOTOR_PORT);
        m_masterLeftMotor = new WPI_TalonSRX(RobotMap.MASTER_LEFT_DRIVE_MOTOR_PORT);
        m_masterRightMotor = new WPI_TalonSRX(RobotMap.MASTER_RIGHT_DRIVE_MOTOR_PORT);

        // Initializes classes to call encoders connected to TalonSRXs
        m_leftDriveEncoder = new SensorCollection(m_masterLeftMotor);
        m_rightDriveEncoder = new SensorCollection(m_masterRightMotor);

        // Zeroes the encoder positions on the drivetrain (connected to TalonSRX)
        m_leftDriveEncoder.setQuadraturePosition(0, 0);
        m_rightDriveEncoder.setQuadraturePosition(0, 0);

        // Sets VictorSPXs to follow TalonSRXs output
        m_slaveLeftMotor.follow(m_masterLeftMotor);
        m_slaveRightMotor.follow(m_masterRightMotor);
        
        // Initializes the drivetrain with the TalonSRX  as the Motors (VictorSPX follows TalonSRX output)
        m_drivetrain = new DifferentialDrive(m_masterLeftMotor, m_masterRightMotor);

        // Initializes feedback variables for speed setter and rotate setter
        // Setters use variables as feedback in order to "ramp" the output gradually
        m_currentSpeed = 0;
        m_currentRotate = 0;

        // Instantiates the NavX
        try {
			m_ahrs = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
        }
        
        // Initializes rotate PID controller with the PIDF constants, the ahrs, the PID output, and the loop time (s)
        m_rotController = new PIDController(RobotMap.P_ROTATE_CONTROLLER, RobotMap.I_ROTATE_CONTROLLER, RobotMap.D_ROTATE_CONTROLLER, RobotMap.F_ROTATE_CONTROLLER, m_ahrs, this, 0.02);
        m_rotController.setInputRange(-180.00f, 180.00f);
        // These values are temporary and need to be changed based on testing
        m_rotController.setOutputRange(-0.7, 0.7);
        m_rotController.setAbsoluteTolerance(RobotMap.TOLERANCE_ROTATE_CONTROLLER);
        m_rotController.setContinuous();
        m_rotController.disable();
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
        if (desiredSpeed > (m_currentSpeed + RobotMap.DRIVE_MAX_DELTA_SPEED)) {
            m_currentSpeed += RobotMap.DRIVE_MAX_DELTA_SPEED;
        }
        // If desired speed is less than current speed by a margin larger than kMaxDeltaSpeed
        // Decrease current speed by kMaxDeltaSpeed's amount
        else if (desiredSpeed < (m_currentSpeed - RobotMap.DRIVE_MAX_DELTA_SPEED)) {
            m_currentSpeed -= RobotMap.DRIVE_MAX_DELTA_SPEED;
        }
        // If desired speed is within kMaxDeltaSpeed's margin to current speed,
        // Set current speed to match desired speed
        else {
            m_currentSpeed = desiredSpeed;
        }

        // If desired rotate is higher than current rotate by a margin larger than kMaxDeltaSpeed,
        // Increase current rotate by kMaxDelaSpeed's amount
        if (desiredRotate > (m_currentRotate + RobotMap.DRIVE_MAX_DELTA_SPEED)) {
            m_currentRotate += RobotMap.DRIVE_MAX_DELTA_SPEED;
        }
        // If desired rotate is less than current rotate by a margin larger than kMaxDeltaSpeed
        // Decrease current rotate by kMaxDeltaSpeed's amount
        else if (desiredRotate < (m_currentRotate - RobotMap.DRIVE_MAX_DELTA_SPEED)) {
            m_currentRotate -= RobotMap.DRIVE_MAX_DELTA_SPEED;
        }
        // If desired rotate is within kMaxDeltaSpeed's margin to current rotate,
        // Set current rotate to match desired speed
        else {
            m_currentRotate = desiredRotate;
        }

        if ((m_currentSpeed < RobotMap.DRIVE_MAX_QUICK_TURN_SPEED) && (m_currentSpeed > RobotMap.DRIVE_MAX_QUICK_TURN_SPEED)) {
            m_quickTurnEnabled = true;
        }
        else {
            m_quickTurnEnabled = false;
        }

        // Pass current speed, current rotate, and the quick turn boolean into the Differential Drive's curatureDrive method
        m_drivetrain.curvatureDrive(m_currentSpeed, m_currentRotate, m_quickTurnEnabled);
    }

    /**
     * Rotates to a set angle without moving forward utilizing the PID and AHRS
     * @param targetAngle The angle you want the robot to rotate to
     * @return Returns true if the PID returns a value low enough that the robot doesn't move (thus finished)
     */
    public boolean rotateToAngle(double targetAngle) {
        // Flag for checking if the method is finished
        boolean isFinished = false;
        
        // Resets the PID only on first entry
        if(m_firstCall){
            // Resets the error
            m_rotController.reset();

            // Enables the PID
            m_rotController.enable();
            
            // Sets the target to our target angle
            m_rotController.setSetpoint(targetAngle);

            // Prevents us from repeating the reset until we run the method again seperately
            m_firstCall = false;
        }

        // Sets our rotate speed to the return of the PID
        double returnedRotate = m_rotController.get();

        // Runs the drivetrain with 0 speed and the rotate speed set by the PID
        curvatureDrive(0, returnedRotate);

        // Checks to see if the the PID is finished or close enough
        if ( (returnedRotate < RobotMap.FINISHED_PID_THRESHOLD) && (returnedRotate > -RobotMap.FINISHED_PID_THRESHOLD) ) {
            isFinished = true;
            m_firstCall = true;
        }

        return isFinished;
    }

    /**
     * Returns the encoder position of the drivetrain left side encoder
     * @return The position of the left side encoder
     */
    public int getLeftDriveEncoderPosition() {
        return m_leftDriveEncoder.getQuadraturePosition();
    }

    /**
     * Returns the encoder position of the drivetrain right side encoder
     * @return The position of the right side encoder
     */
    public int getRightDriveEncoderPosition() {
        return m_rightDriveEncoder.getQuadraturePosition();
    }

    /**
     * Returns the encoder velocity of the drivetrain left side encoder
     * @return The velocity of the left side encoder
     */
    public int getLeftDriveEncoderVelocity() {
        return m_leftDriveEncoder.getQuadratureVelocity();
    }

    /**
     * Returns the encoder velocity of the drivetrain right side encoder
     * @return The velocity of the right side encoder
     */
    public int getRightDriveEncoderVelocity() {
        return m_rightDriveEncoder.getQuadratureVelocity();
    }

    /**
     * 
     */
    public void pidWrite(double output){

    }
}
