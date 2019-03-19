package frc.robot;

/**
 * This class is designed to map out the port numbers for motor controllers,
 * sensors, and other objects on the robot that require a dedicated port on the
 * RoboRIO.
 * @version Week 6 Pre-comp
 */
public class RobotMap {

    // How far off course the robot can go while moving
    // straight before running its rotate method again
    public static final int STRAIGHT_ANGLE_THRESHOLD = 5;

    // Debug telmetry enable / disable constants
    public static final boolean ULTRASONIC_TELEMETRY = false;
    public static final boolean DRIVETRAIN_TELEMETRY = false;
    public static final boolean ELEVATOR_TELEMETRY = false;
    public static final boolean CLIMBER_TELEMETRY = false;
    public static final int SAMPLE_RATE = 50;

    //Auton SendableChooser constants
    public static final String LEFT_AUTO = "Left Auton";
    public static final String RIGHT_AUTO = "Right Auton";
    public static final String TELEOP = "Teleop / Manual";

    // Climber gains
    public static final Gains CLIMBER_GAINS = new Gains(0.3, 0.0, 0.0, 0.0, 100, 1.0);

    // Drivetrain gains
    public static final Gains DRIVETRAIN_GAINS = new Gains(0.3, 0.0, 0.0, 0.0, 100, 1.0);

    // Elevator gains
    public static final Gains ELEVATOR_GAINS = new Gains(0.7, 0.001, 0.0, 0.0, 2, 1.0);

	// NavX angle offset
    public static final int ANGLE_OFFSET = 180;
    // PI constant
    public static final double PI = 3.14159265359;
    // Encoder ticks / revolution
    public static final int TICKS_PER_REVOLUTION = 4096;
    // Controller deadbands
    public static final double CONTROLLER_STICK_DEADBAND = 0.15;
    public static final double CONTROLLER_TRIGGER_DEADBAND = 0.1;
    // Elevator drum measurements
    public static final double DRUM_CIRCUMFERENCE = 8.1875;
    // PID timout constant
    public static final int TIMEOUT_MS = 30;
    // PID Index, 0 is primary, 1 is aux
    public static final int PID_PRIMARY = 0;
    // PID deadband
    public static final double NEUTRAL_DEADBAND = 0.001;
    // PID Peak output
    public static final double PID_PEAK_OUTPUT = 1.0;

    // Controllers
    // Controller ports
    public static final int PILOT_CONTROLLER_PORT = 0;
    public static final int COPILOT_CONTROLLER_PORT = 1;

    // CAN motor controller ID numbers
    // Drivetrain
    // NOTE: Test robot can IDs for drivetrain.

    public static final boolean COMP = false;
    
    // COMP BOT VALUES
    public static final int SLAVE_LEFT_DRIVE_MOTOR_PORT = 12;
    public static final int SLAVE_RIGHT_DRIVE_MOTOR_PORT = 11;
    public static final int MASTER_LEFT_DRIVE_MOTOR_PORT = 2;
    public static final int MASTER_RIGHT_DRIVE_MOTOR_PORT = 1;

    // Elevator
    public static final int ELEVATOR_MOTOR_PORT = 3;
    //Climber back motor
    public static final int CLIMBER_DRIVE_MOTOR_PORT = 16;

	// PWM motor controller port numbers
	// Climber
	public static final int FRONT_CLIMBER_MOTOR_PORT = 5;
	public static final int BACK_CLIMBER_MOTOR_PORT = 4;

    // Sensors
    // Front Climber limit switches DIO port numbers
    public static final int FRONT_CLIMBER_LIMIT_TOP_PORT = 5;
    public static final int FRONT_CLIMBER_LIMIT_BOTTOM_PORT = 6;
    // Back Climber limit switches DIO Port number
    public static final int BACK_CLIMBER_LIMIT_TOP_PORT = 7;
    public static final int BACK_CLIMBER_LIMIT_BOTTOM_PORT = 8;
    // Hatch mech encoder
    public static final int HATCH_MECH_ENCODER_A = 6;
    public static final int HATCH_MECH_ENCODER_B = 7;

    // Servo PWM ports
    // HatchMech
    public static final int HATCH_MECH_SERVO_PORT = 2;

    // Position constants
    // HatchMech servo positions
    public static final double HATCH_MECH_OPEN_SERVO_POSITION = 0.3;
    public static final double HATCH_MECH_CLOSE_SERVO_POSITION = 0.69;
    public static final double HATCH_MECH_DIAGONAL_SERVO_POSITION = 0.5;
    // HatchMech motor encoder limits
    public static final int HATCH_MECH_UP_STOP_POSITION = 50000;
    public static final int HATCH_MECH_DOWN_STOP_POSITION = 25000;
    public static final int HATCH_MECH_MOTOR_PORT = 15;

    // Motor speed constants
    // Drivetrain
    public static final double DRIVE_MAX_DELTA_SPEED = 0.1;
    public static final double DRIVE_MAX_QUICK_TURN_SPEED = 0.1;
    //HatchMech motor speeds
    public static final double HATCH_MECH_ARM_UP_MOTOR_SPEED = 0.5;     // Check Speed
    public static final double HATCH_MECH_ARM_DOWN_MOTOR_SPEED = -0.3;  // Check Speed
    public static final double HATCH_MECH_STOP_MOTOR_SPEED = 0.0;
    // Climber motor speeds
    public static final double FRONT_CLIMBER_SPEED_UP = 0.75; 
    public static final double FRONT_CLIMBER_SPEED_DOWN = -0.9;
    
    public static final double BACK_CLIMBER_SPEED_UP = 0.35;
    public static final double BACK_CLIMBER_SPEED_UP_FAST = 0.75;
    public static final double BACK_CLIMBER_SPEED_DOWN = -0.8;

    // Climber encoder target in tics
    public static final int CLIMBER_TARGET = -550000;

    // Elevator Motor Speed
    public static final double ELEVATOR_MOTOR_SPEED_UP = 0.4;
    public static final double ELEVATOR_MOTOR_SPEED_DOWN = -0.4;
    // Climber back motor speed settings
    public static final double CLIMBER_DRIVE_SPEED_FOREWARD = 0.4;
    public static final double CLIMBER_DRIVE_SPEED_BACKWARD = -0.4;

    // PID Controller
    // Rotate Controller
    public static final double P_ROTATE_CONTROLLER = 0.03;
    public static final double I_ROTATE_CONTROLLER = 0.0002;
    public static final double D_ROTATE_CONTROLLER = 0.07;
    public static final double F_ROTATE_CONTROLLER = 0.00;
    public static final double TOLERANCE_ROTATE_CONTROLLER = .5;

    public static final double P_ROTATE_DRIVE_CONTROLLER = 0.04;
    public static final double I_ROTATE_DRIVE_CONTROLLER = 0.001;
    public static final double D_ROTATE_DRIVE_CONTROLLER = 0.12;
    public static final double F_ROTATE_DRIVE_CONTROLLER = 0.00;
    public static final double TOLERANCE_ROTATE_DRIVE_CONTROLLER = 2;

    public static final double FINISHED_PID_THRESHOLD = 0.1;
    public static final double PID_LOOP_TIME_S = 0.02;
    public static final double PID_INPUT_RANGE = 180.00;
    public static final double PID_OUTPUT_RANGE = 0.5;

    // Constants for calculating drive distance
    public static final double DRIVE_TICS_PER_INCH = (4096 / (6*RobotMap.PI));

    // Speed of robot drive in autonomous functions
    public static final double AUTO_SPEED = 0.35;

    // Constants for climber targets
    public static final int RAISED_CLIMBER_POS = 0;

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

    // Closed loop time of PID systems in milliseconds
    public final static int CLOSED_LOOP_TIME = 10;

    // Cruise and acceleration values for motion magic
    // TODO: Update with testing
    public final static int DRIVE_CRUISE_VELOCITY = 2000;
    public final static int DRIVE_ACCELERATION = 2000;

    public final static int CLIMB_CRUISE_VELOCITY = 6000;
    public final static int CLIMB_ACCELERATION = 2000;

    public final static int ELEVATOR_CRUISE_VELOCITY = 2000;
    public final static int ELEVATOR_ACCELERATION = 2000;

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
    public final static int ENCODER_UNITS_PER_ROTATION = 36224;

    /**
     * PID Gains may have to be adjusted based on the responsiveness of control
     * loop.
     * 
     * Not all set of Gains are used in this project and may be removed as desired.
     * kP kI kD kF Iz PeakOut
     */
    public final static Gains GAINS_TURNING = new Gains(0.1, 0.0, 0.0, 0.0, 200, 1.0);
    public final static Gains GAINS_VELOCIT = new Gains(0.1, 0.0, 0.0, 0.0, 300, 1.0);

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

    //	Declares command constants for arduino communication
	public final static char GET_DEG_TO_TARGET = '2';
	public final static char GET_DIST_TO_TARGET = '1';
	public final static char GET_ANGLE_TO_CENTER = '3';
    public final static char GET_LOW_POSITION = '4';
    public final static char GET_AVG_AREA = '5';
}