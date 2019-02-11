package frc.robot;

import edu.wpi.first.wpilibj.PIDOutput;
// Imports needed for motor controllers, speed controller groups, and the drivetrain
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// These imports are extending SpeedController, allowing us to use SpeedControllerGroup
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorCollection;

// Stolen imports from the CTRE sample code
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

// Import needed to initialize NavX and rotation controller
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SPI;

/**
 * This class defines the main drivetrain used with CAN motor controllers
 * @author Matt, Josh
 * @version Week 5 Pre-comp
 */
public class Drivetrain implements PIDOutput {
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
    private WPI_VictorSPX m_frontLeftMotor;
    private WPI_VictorSPX m_frontRightMotor;
    private WPI_TalonSRX m_backLeftMotor;
    private WPI_TalonSRX m_backRightMotor;

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
        m_frontLeftMotor = new WPI_VictorSPX(RobotMap.FRONT_LEFT_DRIVE_MOTOR_PORT);
        m_frontRightMotor = new WPI_VictorSPX(RobotMap.FRONT_RIGHT_DRIVE_MOTOR_PORT);
        m_backLeftMotor = new WPI_TalonSRX(RobotMap.BACK_LEFT_DRIVE_MOTOR_PORT);
        m_backRightMotor = new WPI_TalonSRX(RobotMap.BACK_RIGHT_DROVE_MOTOR_PORT);

        // Initializes classes to call encoders connected to TalonSRXs
        m_leftDriveEncoder = new SensorCollection(m_backLeftMotor);
        m_rightDriveEncoder = new SensorCollection(m_backRightMotor);

        // Zeroes the encoder positions on the drivetrain (connected to TalonSRX)
        m_leftDriveEncoder.setQuadraturePosition(0, 0);
        m_rightDriveEncoder.setQuadraturePosition(0, 0);

        // Sets VictorSPXs to follow TalonSRXs output
        m_frontLeftMotor.follow(m_backLeftMotor);
        m_frontRightMotor.follow(m_backRightMotor);

        // Initializes the drivetrain with the TalonSRX as the Motors (VictorSPX follows
        // TalonSRX output)
        m_drivetrain = new DifferentialDrive(m_backLeftMotor, m_backRightMotor);

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
     * || Currently in place if access is needed to DifferentialDrive methods || ||
     * Might be used for initSendable or other methods within class || Gets the
     * drivetrain object to use DifferentialDrive methods
     * 
     * @return The drivetrain object (DifferentialDrive)
     */
    public DifferentialDrive getDrivetrain() {
        return m_drivetrain;
    }

    /**
     * Sets the drivetrain motor to desired settings, Acceleration limiter is
     * implemented to prevent current spikes from putting robot in brownout
     * condition
     * 
     * @param desiredSpeed  The desired robot speed along x-axis [-1.0..1.0] forward
     *                      is positive
     * @param desiredRotate The desired robot turning speed along z-axis [-1.0..1.0]
     *                      clockwise is positive
     */
    public void curvatureDrive(double desiredSpeed, double desiredRotate) {

        // If desired speed is higher than current speed by a margin larger than
        // kMaxDeltaSpeed,
        // Increase current speed by kMaxDelaSpeed's amount
        if (desiredSpeed > (m_currentSpeed + RobotMap.DRIVE_MAX_DELTA_SPEED)) {
            m_currentSpeed += RobotMap.DRIVE_MAX_DELTA_SPEED;
        }
        // If desired speed is less than current speed by a margin larger than
        // kMaxDeltaSpeed
        // Decrease current speed by kMaxDeltaSpeed's amount
        else if (desiredSpeed < (m_currentSpeed - RobotMap.DRIVE_MAX_DELTA_SPEED)) {
            m_currentSpeed -= RobotMap.DRIVE_MAX_DELTA_SPEED;
        }
        // If desired speed is within kMaxDeltaSpeed's margin to current speed,
        // Set current speed to match desired speed
        else {
            m_currentSpeed = desiredSpeed;
        }

        // If desired rotate is higher than current rotate by a margin larger than
        // kMaxDeltaSpeed,
        // Increase current rotate by kMaxDelaSpeed's amount
        if (desiredRotate > (m_currentRotate + RobotMap.DRIVE_MAX_DELTA_SPEED)) {
            m_currentRotate += RobotMap.DRIVE_MAX_DELTA_SPEED;
        }
        // If desired rotate is less than current rotate by a margin larger than
        // kMaxDeltaSpeed
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
        } else {
            m_quickTurnEnabled = false;
        }

        // Pass current speed, current rotate, and the quick turn boolean into the
        // Differential Drive's curatureDrive method
        m_drivetrain.curvatureDrive(m_currentSpeed, m_currentRotate, m_quickTurnEnabled);
    }

    /**
     * Rotates to a set angle without moving forward utilizing the PID and AHRS
     * 
     * @param targetAngle The angle you want the robot to rotate to
     * @return Returns true if the PID returns a value low enough that the robot
     *         doesn't move (thus finished)
     */
    public boolean rotateToAngle(double targetAngle) {
        // Flag for checking if the method is finished
        boolean isFinished = false;

        // Resets the PID only on first entry
        if (m_firstCall) {
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
     * 
     * @return
     */
    public void talonDriveConfig() {
        // Sets all motor controllers to zero to kill movement
        m_backLeftMotor.set(ControlMode.PercentOutput, 0);
        m_backRightMotor.set(ControlMode.PercentOutput, 0);

        // Sets all motors to brake
        m_backLeftMotor.setNeutralMode(NeutralMode.Brake);
        m_backRightMotor.setNeutralMode(NeutralMode.Brake);

        /** Feedback Sensor Configuration */

        /* Configure the left Talon's selected sensor to a Quad Encoder */
        m_backLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, // Local Feedback Source

                RobotMap.PID_PRIMARY, // PID Slot for Source [0, 1]

                RobotMap.TIMEOUT_MS); // Configuration Timeout

        /*
         * Configure the Remote Talon's selected sensor as a remote sensor for the right
         * Talon
         */

        m_backRightMotor.configRemoteFeedbackFilter(m_backLeftMotor.getDeviceID(), // Device ID of Source

                RemoteSensorSource.TalonSRX_SelectedSensor, // Remote Feedback Source

                RobotMap.REMOTE_1, // Source number [0, 1]

                RobotMap.TIMEOUT_MS); // Configuration Timeout

        /* Setup Sum signal to be used for Distance */

        m_backRightMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT_MS); // Feedback
                                                                                                                // Device
                                                                                                                // of
                                                                                                                // Remote
                                                                                                                // Talon

        m_backRightMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, RobotMap.TIMEOUT_MS); // Quadrature
                                                                                                              // Encoder
                                                                                                              // of
                                                                                                              // current
                                                                                                              // Talon

        /* Setup Difference signal to be used for Turn */

        m_backRightMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT_MS);

        m_backRightMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, RobotMap.TIMEOUT_MS);

        /* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */

        m_backRightMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorSum,

                RobotMap.PID_PRIMARY,

                RobotMap.TIMEOUT_MS);

        /* Scale Feedback by 0.5 to half the sum of Distance */

        m_backRightMotor.configSelectedFeedbackCoefficient(0.5, // Coefficient

                RobotMap.PID_PRIMARY, // PID Slot of Source

                RobotMap.TIMEOUT_MS); // Configuration Timeout

        /*
         * Configure Difference [Difference between both QuadEncoders] to be used for
         * Auxiliary PID Index
         */

        m_backRightMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference,

                RobotMap.PID_TURN,

                RobotMap.TIMEOUT_MS);

        /* Don't scale the Feedback Sensor (use 1 for 1:1 ratio) */

        m_backRightMotor.configSelectedFeedbackCoefficient(1, RobotMap.PID_TURN, RobotMap.TIMEOUT_MS);

        /* Configure output and sensor direction */

        m_backLeftMotor.setInverted(false);

        m_backLeftMotor.setSensorPhase(true);

        m_backRightMotor.setInverted(true);

        m_backRightMotor.setSensorPhase(true);

        /* Set status frame periods to ensure we don't have stale data */

        m_backRightMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.TIMEOUT_MS);

        m_backRightMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.TIMEOUT_MS);

        m_backRightMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.TIMEOUT_MS);

        m_backLeftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.TIMEOUT_MS);

        /* Configure neutral deadband */

        m_backRightMotor.configNeutralDeadband(RobotMap.kNeutralDeadband, RobotMap.TIMEOUT_MS);

        m_backLeftMotor.configNeutralDeadband(RobotMap.kNeutralDeadband, RobotMap.TIMEOUT_MS);

        /**
         * 
         * Max out the peak output (for all modes).
         * 
         * However you can limit the output of a given PID object with
         * configClosedLoopPeakOutput().
         * 
         */

        m_backLeftMotor.configPeakOutputForward(+1.0, RobotMap.TIMEOUT_MS);

        m_backLeftMotor.configPeakOutputReverse(-1.0, RobotMap.TIMEOUT_MS);

        m_backRightMotor.configPeakOutputForward(+1.0, RobotMap.TIMEOUT_MS);

        m_backRightMotor.configPeakOutputReverse(-1.0, RobotMap.TIMEOUT_MS);

        /* FPID Gains for velocity servo */

        m_backRightMotor.config_kP(RobotMap.kSlot_Velocit, RobotMap.kGains_Velocit.kP, RobotMap.TIMEOUT_MS);

        m_backRightMotor.config_kI(RobotMap.kSlot_Velocit, RobotMap.kGains_Velocit.kI, RobotMap.TIMEOUT_MS);

        m_backRightMotor.config_kD(RobotMap.kSlot_Velocit, RobotMap.kGains_Velocit.kD, RobotMap.TIMEOUT_MS);

        m_backRightMotor.config_kF(RobotMap.kSlot_Velocit, RobotMap.kGains_Velocit.kF, RobotMap.TIMEOUT_MS);

        m_backRightMotor.config_IntegralZone(RobotMap.kSlot_Velocit, RobotMap.kGains_Velocit.kIzone,
                RobotMap.TIMEOUT_MS);

        m_backRightMotor.configClosedLoopPeakOutput(RobotMap.kSlot_Velocit, RobotMap.kGains_Velocit.kPeakOutput,
                RobotMap.TIMEOUT_MS);

        m_backRightMotor.configAllowableClosedloopError(RobotMap.kSlot_Velocit, 0, RobotMap.TIMEOUT_MS);

        /* FPID Gains for turn servo */

        m_backRightMotor.config_kP(RobotMap.kSlot_Turning, RobotMap.kGains_Turning.kP, RobotMap.TIMEOUT_MS);

        m_backRightMotor.config_kI(RobotMap.kSlot_Turning, RobotMap.kGains_Turning.kI, RobotMap.TIMEOUT_MS);

        m_backRightMotor.config_kD(RobotMap.kSlot_Turning, RobotMap.kGains_Turning.kD, RobotMap.TIMEOUT_MS);

        m_backRightMotor.config_kF(RobotMap.kSlot_Turning, RobotMap.kGains_Turning.kF, RobotMap.TIMEOUT_MS);

        m_backRightMotor.config_IntegralZone(RobotMap.kSlot_Turning, RobotMap.kGains_Turning.kIzone,
                RobotMap.TIMEOUT_MS);

        m_backRightMotor.configClosedLoopPeakOutput(RobotMap.kSlot_Turning, RobotMap.kGains_Turning.kPeakOutput,
                RobotMap.TIMEOUT_MS);

        m_backRightMotor.configAllowableClosedloopError(RobotMap.kSlot_Turning, 0, RobotMap.TIMEOUT_MS);

        /**
         * 
         * 1ms per loop. PID loop can be slowed down if need be.
         * 
         * For example,
         * 
         * - if sensor updates are too slow
         * 
         * - sensor deltas are very small per update, so derivative error never gets
         * large enough to be useful.
         * 
         * - sensor movement is very slow causing the derivative error to be near zero.
         * 
         */

        int closedLoopTimeMs = 1;

        m_backRightMotor.configClosedLoopPeriod(0, closedLoopTimeMs, RobotMap.TIMEOUT_MS);

        m_backRightMotor.configClosedLoopPeriod(1, closedLoopTimeMs, RobotMap.TIMEOUT_MS);

        /**
         * 
         * configAuxPIDPolarity(boolean invert, int timeoutMs)
         * 
         * false means talon's local output is PID0 + PID1, and other side Talon is PID0
         * - PID1
         * 
         * true means talon's local output is PID0 - PID1, and other side Talon is PID0
         * + PID1
         * 
         */

        m_backRightMotor.configAuxPIDPolarity(false, RobotMap.TIMEOUT_MS);

    }

    public void talonArcadeDrive (double forward, double turn) {
        m_backLeftMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
        m_backLeftMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
    }

    /**
     * Returns the encoder position of the drivetrain left side encoder
     * 
     * @return The position of the left side encoder
     */
    public int getLeftDriveEncoderPosition() {
        return m_leftDriveEncoder.getQuadraturePosition();
    }

    /**
     * Returns the encoder position of the drivetrain right side encoder
     * 
     * @return The position of the right side encoder
     */
    public int getRightDriveEncoderPosition() {
        return m_rightDriveEncoder.getQuadraturePosition();
    }

    /**
     * Returns the encoder velocity of the drivetrain left side encoder
     * 
     * @return The velocity of the left side encoder
     */
    public int getLeftDriveEncoderVelocity() {
        return m_leftDriveEncoder.getQuadratureVelocity();
    }

    /**
     * Returns the encoder velocity of the drivetrain right side encoder
     * 
     * @return The velocity of the right side encoder
     */
    public int getRightDriveEncoderVelocity() {
        return m_rightDriveEncoder.getQuadratureVelocity();
    }

    /**
     * 
     */
    public void pidWrite(double output) {

    }
}
