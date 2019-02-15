package frc.robot;

/**
 * This class is designed to map out the port numbers for motor controllers,
 * sensors, and other objects on the robot that require a dedicated port on the
 * RoboRIO.
 * 
 * @version Week 5 Pre-comp
 */
public class RobotMap {

    // Math constants
    // NavX angle offset
    public static final int ANGLE_OFFSET = 180;

    // Controllers
    // Controller ports
    public static final int PILOT_CONTROLLER_PORT = 0;
    public static final int COPILOT_CONTROLLER_PORT = 1;
    // Controller deadbands
    public static final double CONTROLLER_STICK_DEADBAND = 0.2;
    public static final double CONTROLLER_TRIGGER_DEADBAND = 0.05;

    // CAN motor controller ID numbers
    // Drivetrain
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 3;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 1;
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 4;
    public static final int BACK_RIGHT_DROVE_MOTOR_PORT = 2;
    // Elevator
    public static final int ELEVATOR_MOTOR_PORT = 4;
    public static final double DRUM_DIAMETER_INCHES = 2.2; // UPDATE THIS VALUE
    public static final double PI = 3.14159265359;
    public static final double DRUM_CIRCUMFERENCE = DRUM_DIAMETER_INCHES * PI;
    public static final int TICKS_PER_REVOLUTION = 4096;

    // PWM motor controller port numbers
    // Climber
    public static final int FRONT_CLIMBER_MOTOR_PORT = 0;
    public static final int BACK_CLIMBER_MOTOR_PORT = 1;

    // Sensors
    // Elevator limit switches DIO port numbers
    public static final int ELEVATOR_LIMIT_TOP_PORT = 0;
    public static final int ELEVATOR_LIMIT_BOTTOM_PORT = 1;
    // Climber limit switches DIO port numbers
    public static final int FRONT_CLIMBER_LIMIT_TOP_PORT = 2;
    public static final int BACK_CLIMBER_LIMIT_TOP_PORT = 3;
    // Hatch mechanism limit switches
    public static final int HATCH_MECH_LIMIT_TOP_PORT = 1;
    public static final int HATCH_MECH_LIMIT_BOTTOM_PORT = 2;

    // Servo PWM ports
    // HatchMech
    public static final int HATCH_MECH_SERVO_PORT = 0;

    // Position constants
    // HatchMech servo positions
    public static final double HATCH_MECH_OPEN_SERVO_POSITION = 0.6;
    public static final double HATCH_MECH_CLOSE_SERVO_POSITION = 0.3;
    // Elevator height
    public static final double MAX_ELEVATOR_HEIGHT = 5.0;

    // Motor speed constants
    // Drivetrain
    public static final double DRIVE_MAX_DELTA_SPEED = 0.1;
    public static final double DRIVE_MAX_QUICK_TURN_SPEED = 0.1;
    // HatchMech motor speeds
    public static final double HATCH_MECH_ARM_UP_MOTOR_SPEED = 0.5;
    public static final double HATCH_MECH_ARM_DOWN_MOTOR_SPEED = -0.5;
    public static final double HATCH_MECH_STOP_MOTOR_SPEED = 0.0;
    // Climber motor speeds
    public static final double CLIMBER_SPEED_UP = 0.3; // UPDATE THIS VALUE
    public static final double CLIMBER_SPEED_DOWN = -0.3; // UPDATE THIS VALUE

    // PID Controller
    // Rotate Controller
    public static final double P_ROTATE_CONTROLLER = 0.00;
    public static final double I_ROTATE_CONTROLLER = 0.00;
    public static final double D_ROTATE_CONTROLLER = 0.00;
    public static final double F_ROTATE_CONTROLLER = 0.00;
    public static final double TOLERANCE_ROTATE_CONTROLLER = 2;
    public static final double FINISHED_PID_THRESHOLD = 0.01;

    // Stolen constants for sample code

    /**
     * 
     * Using the configSelectedFeedbackCoefficient() function, scale units to 3600
     * per rotation.
     * 
     * This is nice as it keeps 0.1 degrees of resolution, and is fairly intuitive.
     * 
     */
    public final static double TURN_TRAVEL_UNITS_PER_ROTATION = 3600;

    /**
     * Empirically measure what the difference between encoders per 360'
     * 
     * Drive the robot in clockwise rotations and measure the units per rotation.
     * 
     * Drive the robot in counter clockwise rotations and measure the units per rotation.
     * 
     * Take the average of the two.
     * 
     * Josh's note: This is referring to the actual robot, which means we need to test with said robot.
     * This cannot be used without that testing
     */
    public final static int ENCODER_UNITS_PER_ROTATION = 51711;

    /**
     * Set to zero to skip waiting for confirmation.
     * 
     * Set to nonzero to wait and report to DS if action fails.
     */

    public final static int TIMEOUT_MS = 30;

    // Motor neutral dead-band, set to the minimum 0.1%.
    public final static double NEUTRAL_DEADBAND = 0.001;

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control
     * loop.
     * 
     * kF: 1023 represents output value to Talon at 100%, 6800 represents Velocity
     * units at 100% output
     * 
     * Not all set of Gains are used in this project and may be removed as desired.
     * kP kI kD kF Iz PeakOut
     */
    public final static Gains GAINS_DISTANC = new Gains(0.1, 0.0, 0.0, 0.0, 100, 0.50);
    public final static Gains GAINS_TURNING = new Gains(2.0, 0.0, 4.0, 0.0, 200, 1.00);
    public final static Gains GAINS_VELOCIT = new Gains(0.1, 0.0, 20.0, 1023.0 / 6800.0, 300, 0.50);
    public final static Gains GAINS_MOT_PROF = new Gains(1.0, 0.0, 0.0, 1023.0 / 6800.0, 400, 1.00);

    /** ---- Flat constants, you should not need to change these ---- */

    /*
     * We allow either a 0 or 1 when selecting an ordinal for remote devices [You
     * can have up to 2 devices assigned remotely to a talon/victor]
     */
    public final static int REMOTE_0 = 0;
    public final static int REMOTE_1 = 1;

    /*
     * We allow either a 0 or 1 when selecting a PID Index, where 0 is primary and 1
     * is auxiliary
     */
    public final static int PID_PRIMARY = 0;
    public final static int PID_TURN = 1;

    /*
     * Firmware currently supports slots [0, 3] and can be used for either PID Set
     */
    public final static int SLOT_0 = 0;
    public final static int SLOT_1 = 1;
    public final static int SLOT_2 = 2;
    public final static int SLOT_3 = 3;

    /* ---- Named slots, used to clarify code ---- */
    public final static int SLOT_DISTANC = SLOT_0;
    public final static int SLOT_TURNING = SLOT_1;
    public final static int SLOT_VELOCIT = SLOT_2;
    public final static int SLOT_MOT_PROF = SLOT_3;
}