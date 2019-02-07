package frc.robot;

/**
 * This class is designed to map out the port numbers for motor controllers, sensors,
 * and other objects on the robot that require a dedicated port on the RoboRIO.
 */
public class RobotMap {

    // Controllers
    public static final int PILOT_CONTROLLER_PORT = 0;
    public static final int COPILOT_CONTROLLER_PORT = 1;

    // CAN Motor controllers
    // Drivetrain
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 0;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 1;
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 2;
    public static final int BACK_RIGHT_DROVE_MOTOR_PORT = 3;
    // Elevator
    public static final int ELEVATOR_MOTOR_PORT = 4;

    // PWM Motor Controllers
    // Climber
    public static final int FRONT_CLIMBER_MOTOR_PORT = 0;
    public static final int BACK_CLIMBER_MOTOR_PORT = 1;

    // Sensors
    // Elevator limit switches
    public static final int ELEVATOR_LIMIT_TOP_PORT = 0;
    public static final int ELEVATOR_LIMIT_BOTTOM_PORT = 1;
    // Climber limit switches
    public static final int FRONT_CLIMBER_LIMIT_TOP_PORT = 2;
    public static final int BACK_CLIMBER_LIMIT_TOP_PORT = 3;

    //HatchMech ports
    public static final int HATCH_MECH_SERVO_PORT = 0;
    public static final int HATCH_MECH_LIMIT_TOP_PORT = 1;
    public static final int HATCH_MECH_LIMIT_BOTTOM_PORT = 2;

    //HatchMech servo positions
    public static final double HATCH_MECH_OPEN_SERVO_POSITION = 0.6;
    public static final double HATCH_MECH_CLOSE_SERVO_POSITION = 0.3;

    //HatchMech motor speeds
    public static final double HATCH_MECH_ARM_UP_MOTOR_SPEED = 0.5;
    public static final double HATCH_MECH_ARM_DOWN_MOTOR_SPEED = -0.5;
    public static final double HATCH_MECH_STOP_MOTOR_SPEED = 0.0;
}