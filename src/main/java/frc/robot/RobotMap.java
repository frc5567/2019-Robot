package frc.robot;

/**
 * This class is designed to map out the port numbers for motor controllers, sensors,
 * and other objects on the robot that require a dedicated port on the RoboRIO.
 */
public class RobotMap {

    // Controllers
    public static final int PILOT_CONTROLLER_PORT = 0;
    public static final int COPILOT_CONTROLLER_PORT = 1;

    // Motor controllers
    // Drivetrain
    public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 0;
    public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 1;
    public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 2;
    public static final int BACK_RIGHT_DROVE_MOTOR_PORT = 3;
    // Elevator
    public static final int ELEVATOR_MOTOR_PORT = 4;

    // Sensors
    // Elevator limit switches
    public static final int ELEVATOR_LIMIT_TOP_PORT = 0;
    public static final int ELEVATOR_LIMIT_BOTTOM_PORT = 1;
}