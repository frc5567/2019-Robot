package frc.robot;

/**
 * This class is designed to map out the port numbers for motor controllers,
 * sensors, and other objects on the robot that require a dedicated port on the
 * RoboRIO.
 * @version Week 6 Pre-comp
 */
public class RobotMap {

	// NavX angle offset
    public static final int ANGLE_OFFSET = 180;
    // PI constant
    public static final double PI = 3.14159265359;
    // Encoder ticks / revolution
    public static final int TICKS_PER_REVOLUTION = 4096;
    // Controller deadbands
    public static final double CONTROLLER_STICK_DEADBAND = 0.05;
    public static final double CONTROLLER_TRIGGER_DEADBAND = 0.05;
    // Elevator drum measurements
    public static final double DRUM_DIAMETER_INCHES = 2.2;     //UPDATE THIS VALUE
    public static final double DRUM_CIRCUMFERENCE = DRUM_DIAMETER_INCHES * PI;

    // Controllers
    // Controller ports
	public static final int PILOT_CONTROLLER_PORT = 0;
    public static final int COPILOT_CONTROLLER_PORT = 1;

    // CAN motor controller ID numbers
    // Drivetrain
    // NOTE: Test robot can IDs for drivetrain.
    // TODO: Rename variables to reflect Victor or Talon. Front/back tags do not apply
    public static final int SLAVE_LEFT_DRIVE_MOTOR_PORT = 3;
    public static final int SLAVE_RIGHT_DRIVE_MOTOR_PORT = 1;
    public static final int MASTER_LEFT_DRIVE_MOTOR_PORT = 4;
    public static final int MASTER_RIGHT_DRIVE_MOTOR_PORT = 2;
    // Elevator
<<<<<<< HEAD
    public static final int ELEVATOR_MOTOR_PORT = 4;
    public static final double DRUM_DIAMETER_INCHES = 2.2;     //UPDATE THIS VALUE
    public static final double PI = 3.14159265359;
    public static final double DRUM_CIRCUMFERENCE = DRUM_DIAMETER_INCHES * PI;  //roughly ~6.908 inches per revolution
    public static final int TICKS_PER_REVOLUTION = 4096;
=======
    public static final int ELEVATOR_MOTOR_PORT = 5;
>>>>>>> 1dcf4c991d7e78244cdbf1b95120cfbd211926cd

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
    public static final int HATCH_MECH_LIMIT_TOP_PORT = 4;
    public static final int HATCH_MECH_LIMIT_BOTTOM_PORT = 5;
    // Hatch mech encoder
    public static final int HATCH_MECH_ENCODER_A = 6;
    public static final int HATCH_MECH_ENCODER_B = 7;

    // Servo PWM ports
    // HatchMech
    public static final int HATCH_MECH_SERVO_PORT = 0;

    // Position constants
    // HatchMech servo positions
    public static final double HATCH_MECH_OPEN_SERVO_POSITION = 0.6;
    public static final double HATCH_MECH_CLOSE_SERVO_POSITION = 0.3;
    // HatchMech motor encoder limits
    public static final int HATCH_MECH_UP_MOTOR_POSITION = 0;
    public static final int HATCH_MECH_DOWN_MOTOR_POSITION = 60;

    // Elevator height
    public static final double MAX_ELEVATOR_HEIGHT = 5.0;
    public static final double MID_ELEVATOR_HEIGHT = _._;                  //Insert mid height value
    public static final double LOW_ELEVATOR_HEIGHT = _._;                  //Insert mid height value

    // Motor speed constants
    //Drivetrain
    public static final double DRIVE_MAX_DELTA_SPEED = 0.1;
    public static final double DRIVE_MAX_QUICK_TURN_SPEED = 0.1;
    //HatchMech motor speeds
<<<<<<< HEAD
    public static final double HATCH_MECH_ARM_UP_MOTOR_SPEED = 0.5;         //CHECK SPEED
    public static final double HATCH_MECH_ARM_DOWN_MOTOR_SPEED = -0.5;      //CHECK SPEED
=======
    public static final double HATCH_MECH_ARM_UP_MOTOR_SPEED = 0.5;     // Check Speed
    public static final double HATCH_MECH_ARM_DOWN_MOTOR_SPEED = -0.5;  // Check Speed
>>>>>>> 1dcf4c991d7e78244cdbf1b95120cfbd211926cd
    public static final double HATCH_MECH_STOP_MOTOR_SPEED = 0.0;
    // Climber motor speeds
    public static final double CLIMBER_SPEED_UP = 0.3;      //UPDATE THIS VALUE
    public static final double CLIMBER_SPEED_DOWN = -0.3;       //UPDATE THIS VALUE
<<<<<<< HEAD
    //Elevator motor speeds
=======
    // Elevator Motor Speed
>>>>>>> 1dcf4c991d7e78244cdbf1b95120cfbd211926cd
    public static final double ELEVATOR_MOTOR_SPEED_UP = 0.4;
    public static final double ELEVATOR_MOTOR_SPEED_DOWN = -0.4;

    // PID Controller
    // Rotate Controller
    public static final double P_ROTATE_CONTROLLER = 0.00;
    public static final double I_ROTATE_CONTROLLER = 0.00;
    public static final double D_ROTATE_CONTROLLER = 0.00;
    public static final double F_ROTATE_CONTROLLER = 0.00;
    public static final double TOLERANCE_ROTATE_CONTROLLER = 2;
    public static final double FINISHED_PID_THRESHOLD = 0.01;
}