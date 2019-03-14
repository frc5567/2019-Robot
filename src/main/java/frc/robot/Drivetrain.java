package frc.robot;

import edu.wpi.first.wpilibj.PIDOutput;
// Imports needed for motor controllers, speed controller groups, and the drivetrain
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// These imports are extending SpeedController, allowing us to use SpeedControllerGroup
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

// Stolen imports from the CTRE sample code
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;

// Import needed to initialize NavX and rotation controller
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;

/**
 * This class defines the main drivetrain used with CAN motor controllers
 * @author Matt, Josh
 * @version Week 5 Pre-comp
 */
public class Drivetrain implements PIDOutput {
    // Declares NavX for rotation control
    private NavX m_gyro;

    // Declares turn control PID
    PIDController m_rotController;

    // Delcares a flag for checking if this is the first time entering this method in a given run
    boolean m_firstCall = true;

    // Declares variables for current speed and rotate rate
    // Variables used for feedback in speed setter and rotate setter
    double m_currentSpeed;
    double m_currentRotate;

    // Declares variables to enable quick turn when speed is low enough
    boolean m_quickTurnEnabled;

    // Declarations for the motor controllers
    WPI_VictorSPX m_slaveLeftMotor;
    WPI_VictorSPX m_slaveRightMotor;
    WPI_TalonSRX m_masterLeftMotor;
    WPI_TalonSRX m_masterRightMotor;

    // Declaration for encoders connected to TalonSRXs
    private SensorCollection m_leftDriveEncoder;
    private SensorCollection m_rightDriveEncoder;

    // Declaration for the drivetrain
    private DifferentialDrive m_drivetrain;

    // Declaration for ultrasonics
    private Ultrasonic ultraLeft;
    private Ultrasonic ultraRight;

    // Declaration for tic tracking for drive to position
    private double m_leftInitTics;
    private double m_rightInitTics;
    private double m_leftTargetTics;
    private double m_rightTargetTics;
    private double m_ticsToTarget;

    private boolean m_firstCallTest = true;
    private boolean m_mmDriveToPosFirstFlag = true;

    // Counter for buying time for the PID
    int m_counter;

    /**
     * Constructor for the motor controller declarations and the drivetrain object
     */
    public Drivetrain(NavX ahrs) {

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
        m_gyro = ahrs;

        // Instantiates the Ultrasonics
        ultraLeft = new Ultrasonic(2, 1);
        ultraRight = new Ultrasonic(4, 3);
        ultraLeft.setEnabled(true);
        ultraLeft.setAutomaticMode(true);
        ultraLeft.setDistanceUnits(Unit.kInches);
        ultraRight.setEnabled(true);
        ultraRight.setAutomaticMode(true);
        ultraRight.setDistanceUnits(Unit.kInches);
        
        // Initializes rotate PID controller with the PIDF constants, the ahrs, the PID output, and the loop time (s)
        m_rotController = new PIDController(RobotMap.P_ROTATE_CONTROLLER, RobotMap.I_ROTATE_CONTROLLER, RobotMap.D_ROTATE_CONTROLLER, RobotMap.F_ROTATE_CONTROLLER, m_gyro, this, RobotMap.PID_LOOP_TIME_S);
        m_rotController.setInputRange(-RobotMap.PID_INPUT_RANGE, RobotMap.PID_INPUT_RANGE);
        // These values are temporary and need to be changed based on testing
        m_rotController.setOutputRange(-RobotMap.PID_OUTPUT_RANGE, RobotMap.PID_OUTPUT_RANGE);
        m_rotController.setAbsoluteTolerance(RobotMap.TOLERANCE_ROTATE_CONTROLLER);
        m_rotController.setContinuous();
        m_rotController.disable();

        m_counter = 0;
    }

    /**
     * || Currently in place if access is needed to DifferentialDrive methods || 
     * || Might be used for initSendable or other methods within class        || 
     * Gets the drivetrain object to use DifferentialDrive methods
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

            m_counter = 0;
        }

        // Sets our rotate speed to the return of the PID
        double returnedRotate = m_rotController.get();

        // Runs the drivetrain with 0 speed and the rotate speed set by the PID
        talonArcadeDrive(0, returnedRotate, false);

        // Checks to see if the the PID is finished or close enough
        if ( ((returnedRotate < RobotMap.FINISHED_PID_THRESHOLD) && (returnedRotate > -RobotMap.FINISHED_PID_THRESHOLD)) && (m_counter > 10)) {
            isFinished = true;
            m_firstCall = true;
            System.out.println("FINISHED");
        }

        if (isFinished) {
            m_counter = 0;
        }
        else {
            m_counter++;
        }

        return isFinished;
    }

    public boolean rotateDriveAngle(double targetAngle, double target) {
        // Flag for checking if the method is finished
        boolean isFinished = false;

        if (m_firstCall) {
            // Resets the error
            m_rotController.reset();

            // Enables the PID
            m_rotController.enable();
            // Prevents us from repeating the reset until we run the method again seperately
            m_firstCall = false;
        }

        if (m_rotController.getSetpoint() != targetAngle) {
            // Resets the error
            m_rotController.reset();

            // Enables the PID
            m_rotController.enable();

            // Sets the target to our target angle
            m_rotController.setSetpoint(targetAngle);
        }        

        // Sets our rotate speed to the return of the PID
        double returnedRotate = m_rotController.get();

        System.out.println("Returned Rotate: \t" + returnedRotate);

        // Runs the drivetrain with 0 speed and the rotate speed set by the PID
        talonArcadeDrive(RobotMap.AUTO_SPEED, returnedRotate, false);

        if (ultraLeft.getRangeInches() > target && ultraRight.getRangeInches() > target) {
            talonArcadeDrive(RobotMap.AUTO_SPEED, returnedRotate, false);
            isFinished = false;
        }
        else {
            talonArcadeDrive(0.2, 0, false);
            isFinished = true;
        }

        // isFinished acts as an exit flag once we have fulfilled the conditions desired
        return isFinished;
    }

    /**
     * Method for driving a specific distance in auton
     * @param stopDistance Inches away from the target we want to stop
     */
    public boolean driveToUltra(double stopDistance) {
        if (ultraLeft.getRangeInches() > stopDistance && ultraRight.getRangeInches() > stopDistance) {
            talonArcadeDrive(.5, 0, false);
            return false;
        }
        else {
            talonArcadeDrive(0, 0, false);
            return true;
        }
    }

    /**
     * Runs the teleop init section of the sample code. This method shoud be called there
     * Sets up and configs everything on the talons for arcade drive via velocity PID.
     */
    public void talonDriveConfig() {
        // Sets all motor controllers to zero to kill movement
        m_masterLeftMotor.set(ControlMode.PercentOutput, 0);
        m_masterRightMotor.set(ControlMode.PercentOutput, 0);

        // Sets all motors to brake
        m_masterLeftMotor.setNeutralMode(NeutralMode.Brake);
        m_masterRightMotor.setNeutralMode(NeutralMode.Brake);
        
        m_slaveLeftMotor.setNeutralMode(NeutralMode.Brake);
        m_slaveRightMotor.setNeutralMode(NeutralMode.Brake);

        /** Feedback Sensor Configuration */

        // Configure the left Talon's selected sensor to a Quad Encoder
        m_masterLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);

        // Configure the Remote Talon's selected sensor as a remote sensor for the right Talon
        m_masterRightMotor.configRemoteFeedbackFilter(m_masterLeftMotor.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_1, RobotMap.TIMEOUT_MS);

        // Setup Sum signal to be used for Distance
        // Feedback Device of Remote Talon
        m_masterRightMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT_MS); 

        // Quadrature Encoder of current Talon
        m_masterRightMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.QuadEncoder, RobotMap.TIMEOUT_MS); 

        // Setup Difference signal to be used for Turn
        m_masterRightMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor1, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, RobotMap.TIMEOUT_MS);

        // Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index
        m_masterRightMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);

        // Scale Feedback by 0.5 to half the sum of Distance
        m_masterRightMotor.configSelectedFeedbackCoefficient(0.5, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);

        // Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index
        m_masterRightMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, RobotMap.PID_TURN, RobotMap.TIMEOUT_MS);

        // Don't scale the Feedback Sensor (use 1 for 1:1 ratio)
        m_masterRightMotor.configSelectedFeedbackCoefficient(1, RobotMap.PID_TURN, RobotMap.TIMEOUT_MS);

        m_masterRightMotor.setSelectedSensorPosition(0, 0, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.setSelectedSensorPosition(0, 1, RobotMap.TIMEOUT_MS);
        m_masterLeftMotor.setSelectedSensorPosition(0);
        
        // Configure output and sensor direction
        m_masterLeftMotor.setInverted(false);
        m_masterLeftMotor.setSensorPhase(true);
        m_masterRightMotor.setInverted(false);
        m_masterRightMotor.setSensorPhase(true);


        // Set status frame periods to ensure we don't have stale data
        m_masterRightMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.TIMEOUT_MS);
        m_masterLeftMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.TIMEOUT_MS);

        // Configure neutral deadband
        m_masterRightMotor.configNeutralDeadband(RobotMap.NEUTRAL_DEADBAND, RobotMap.TIMEOUT_MS);
        m_masterLeftMotor.configNeutralDeadband(RobotMap.NEUTRAL_DEADBAND, RobotMap.TIMEOUT_MS);

        /**
         * 
         * Max out the peak output (for all modes).
         * 
         * However you can limit the output of a given PID object with
         * configClosedLoopPeakOutput().
         * 
         */
        m_masterLeftMotor.configPeakOutputForward(+1.0, RobotMap.TIMEOUT_MS);
        m_masterLeftMotor.configPeakOutputReverse(-1.0, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.configPeakOutputForward(+1.0, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.configPeakOutputReverse(-1.0, RobotMap.TIMEOUT_MS);

        // Motion Magic Config
		m_masterRightMotor.configMotionAcceleration(RobotMap.DRIVE_ACCELERATION, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.configMotionCruiseVelocity(RobotMap.DRIVE_CRUISE_VELOCITY, RobotMap.TIMEOUT_MS);
        
        // FPID Gains for velocity servo
        m_masterRightMotor.config_kP(0, RobotMap.GAINS.kP, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.config_kI(0, RobotMap.GAINS.kI, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.config_kD(0, RobotMap.GAINS.kD, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.config_kF(0, RobotMap.GAINS.kF, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.config_IntegralZone(0, RobotMap.GAINS.kIzone, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.configClosedLoopPeakOutput(0, RobotMap.GAINS.kPeakOutput, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.configAllowableClosedloopError(0, 0, RobotMap.TIMEOUT_MS);

        // FPID Gains for turn servo
        m_masterRightMotor.config_kP(1, RobotMap.GAINS_TURNING.kP, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.config_kI(1, RobotMap.GAINS_TURNING.kI, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.config_kD(1, RobotMap.GAINS_TURNING.kD, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.config_kF(1, RobotMap.GAINS_TURNING.kF, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.config_IntegralZone(1, RobotMap.GAINS_TURNING.kIzone, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.configClosedLoopPeakOutput(1, RobotMap.GAINS_TURNING.kPeakOutput, RobotMap.TIMEOUT_MS);
		m_masterRightMotor.configAllowableClosedloopError(1, 0, RobotMap.TIMEOUT_MS);

        m_masterRightMotor.configClosedLoopPeriod(0, RobotMap.CLOSED_LOOP_TIME, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.configClosedLoopPeriod(1, RobotMap.CLOSED_LOOP_TIME, RobotMap.TIMEOUT_MS);

        m_masterRightMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);
       
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

        m_masterRightMotor.configAuxPIDPolarity(false, RobotMap.TIMEOUT_MS);

        // Sets profile slot for PID
        m_masterRightMotor.selectProfileSlot(0, RobotMap.PID_PRIMARY);
        m_masterRightMotor.selectProfileSlot(1, RobotMap.PID_TURN);
    }

    /**
     * An arcade drive using the integrated velocity PID on the talons
     * @param forward -1.0 to 1.0, the speed at which you want the robot to move forward
     * @param turn -1.0 to 1.0, the rate of rotation
     * @param setter If this is true, use speed setters to adjust speed and conserve battery. If false, use raw input
     */
    public void talonArcadeDrive (double forward, double turn, boolean setter) {
        if (setter) {
            // If desired speed is higher than current speed by a margin larger than
            // kMaxDeltaSpeed,
            // Increase current speed by kMaxDelaSpeed's amount
            if (forward > (m_currentSpeed + RobotMap.DRIVE_MAX_DELTA_SPEED)) {
                m_currentSpeed += RobotMap.DRIVE_MAX_DELTA_SPEED;
            }
            // If desired speed is less than current speed by a margin larger than
            // kMaxDeltaSpeed
            // Decrease current speed by kMaxDeltaSpeed's amount
            else if (forward < (m_currentSpeed - RobotMap.DRIVE_MAX_DELTA_SPEED)) {
                m_currentSpeed -= RobotMap.DRIVE_MAX_DELTA_SPEED;
            }
            // If desired speed is within kMaxDeltaSpeed's margin to current speed,
            // Set current speed to match desired speed
            else {
                m_currentSpeed = forward;
            }

            // If desired rotate is higher than current rotate by a margin larger than
            // kMaxDeltaSpeed,
            // Increase current rotate by kMaxDelaSpeed's amount
            if (turn > (m_currentRotate + RobotMap.DRIVE_MAX_DELTA_SPEED)) {
                m_currentRotate += RobotMap.DRIVE_MAX_DELTA_SPEED;
            }
            // If desired rotate is less than current rotate by a margin larger than
            // kMaxDeltaSpeed
            // Decrease current rotate by kMaxDeltaSpeed's amount
            else if (turn < (m_currentRotate - RobotMap.DRIVE_MAX_DELTA_SPEED)) {
                m_currentRotate -= RobotMap.DRIVE_MAX_DELTA_SPEED;
            }
            // If desired rotate is within kMaxDeltaSpeed's margin to current rotate,
            // Set current rotate to match desired speed
            else {
                m_currentRotate = turn;
            }
            m_masterLeftMotor.set(ControlMode.PercentOutput, m_currentRotate, DemandType.ArbitraryFeedForward, +m_currentSpeed);
            m_masterRightMotor.set(ControlMode.PercentOutput, m_currentRotate, DemandType.ArbitraryFeedForward, -m_currentSpeed);
        }
        else {
            m_masterLeftMotor.set(ControlMode.PercentOutput, turn, DemandType.ArbitraryFeedForward, +forward);
            m_masterRightMotor.set(ControlMode.PercentOutput, turn, DemandType.ArbitraryFeedForward, -forward);
        }
    }

    public boolean magicDriveToPosition(double distToTarget) {
        // If the return value is valid, run needed calculation
        if (m_mmDriveToPosFirstFlag) {
            m_ticsToTarget = inToTics(distToTarget);
            m_rightInitTics = getRightDriveEncoderPosition();
            m_rightTargetTics = m_rightInitTics - m_ticsToTarget;
            
            m_mmDriveToPosFirstFlag = false;
            return false;
        }
        else {
            m_masterRightMotor.set(ControlMode.MotionMagic, m_rightTargetTics, DemandType.AuxPID, 0);
            m_masterLeftMotor.follow(m_masterRightMotor, FollowerType.AuxOutput1);

            // TODO: Figure out how to return true exclusivley when finished without early exit
            return true;
        }
    }

    public boolean driveToPosition(double distToTarget) {
        // If the return value is valid, run needed calculation
        if (m_firstCallTest) {
            m_ticsToTarget = inToTics(distToTarget);
            m_leftInitTics = getLeftDriveEncoderPosition();
            m_rightInitTics = getRightDriveEncoderPosition();
            m_leftTargetTics = m_leftInitTics - m_ticsToTarget;
            m_rightTargetTics = m_rightInitTics - m_ticsToTarget;
            
            m_firstCallTest = false;
            return false;
        }
        else {
            if (m_leftTargetTics <= m_leftInitTics) {
                // Drives straight if we have not reached our target
                if (m_leftTargetTics < getLeftDriveEncoderPosition()/* && m_rightTargetTics < getLeftDriveEncoderPosition() */) {
                    talonArcadeDrive(RobotMap.AUTO_SPEED, 0, false);
                    return false;
                }
                else {
                    // Stops the arcade drive otherwise
                    m_firstCallTest = true;
                    talonArcadeDrive(0, 0, false);
                    return true;
                }
            }
            else {
                // Drives straight if we have not reached our target
                if (m_leftTargetTics > getLeftDriveEncoderPosition()/* && m_rightTargetTics > getLeftDriveEncoderPosition() */) {
                    talonArcadeDrive(-RobotMap.AUTO_SPEED, 0, false);
                    return false;
                }
                else {
                    // Stops the arcade drive otherwise
                    m_firstCallTest = true;
                    talonArcadeDrive(0, 0, false);
                    return true;
                }
            }
        }
    }

    /**
     * Converts inches to drive encoder tics
     * @param inches The inches we want to convert
     * @return Returns the resultant tips
     */
    private double inToTics(double inches) {
        return inches*RobotMap.DRIVE_TICS_PER_INCH;
    }

    /**
     * 
     */
    public Ultrasonic getLeftUltra() {
        return ultraLeft;
    }

    /**
     * 
     */
    public Ultrasonic getRightUltra() {
        return ultraRight;
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
