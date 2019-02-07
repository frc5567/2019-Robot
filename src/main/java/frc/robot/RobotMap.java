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
    public static final double DRUM_DIAMETER_INCHES = 0.0;     //UPDATE THIS
    public static final int TICKS_PER_REVOLUTION = 4096;

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
}