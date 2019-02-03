package frc.robot;

import frc.robot.Controller;
import frc.robot.Drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.SensorCollection;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;

public class RobotMap {

    // Controller
    public static Controller m_pilotController;
    //public static Controller copilotController;

    // Drivetrain
    public static WPI_VictorSPX m_frontLeftDriveMotor;
    public static WPI_VictorSPX m_frontRightDriveMotor;

    public static WPI_TalonSRX m_backLeftDriveMotor;
    public static WPI_TalonSRX m_backRightDriveMotor;

    public static SpeedControllerGroup leftDriveMotors;
    public static SpeedControllerGroup rightDriveMotors;

    public static Drivetrain m_drivetrain;

    public static SensorCollection m_leftDriveEncoder;
    public static SensorCollection m_rightDriveEncoder;

    // Elevator
    public static DigitalInput m_elevatorLimitTop;
    public static DigitalInput m_elevatorLimitBottom;

    public static SensorCollection m_elevatorEncoder;

    public static WPI_TalonSRX m_elevatorMotor;

    RobotMap() {

        // Controller mapping
        // Controller ports
        m_pilotController = new Controller(0);
        //copilotController = new Controller(1);

        // Drivetrain mapping
        // Front drive motor CAN IDs
        m_frontLeftDriveMotor = new WPI_VictorSPX(0);
        m_frontRightDriveMotor = new WPI_VictorSPX(1);
        // Back drive motor CAN IDs
        m_backLeftDriveMotor = new WPI_TalonSRX(2);
        m_backRightDriveMotor = new WPI_TalonSRX(3);
        // Drivetrain
        m_drivetrain = new Drivetrain();
        // Sensor Collection classes for ecoder value retrival
        m_leftDriveEncoder = new SensorCollection(m_backLeftDriveMotor);
        m_rightDriveEncoder = new SensorCollection(m_backRightDriveMotor);

        // Drivetrain settings
        m_drivetrain.getDrivetrain().setSafetyEnabled(true);
        m_drivetrain.getDrivetrain().setExpiration(0.1);

        // Elevator mapping
        // Elevator motor controller CAN ID
        m_elevatorMotor = new WPI_TalonSRX(4);
        // Limit switches ports
        m_elevatorLimitTop = new DigitalInput(0);
        m_elevatorLimitBottom = new DigitalInput(1);
        // Sensor Collection class declaration for encoder on the elevator motor controller
        m_elevatorEncoder = new SensorCollection(m_elevatorMotor);
    }
}