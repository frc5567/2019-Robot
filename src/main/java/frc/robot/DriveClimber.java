package frc.robot;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DriveClimber extends Climber {

    // Declares a VictorSPX for the drive motor on the climber
    WPI_VictorSPX m_driveMotor;

    /**
     * Class for the back climber, extends the generic climber class but with drive motor methods added
     * @param motorPort ID of the climber raise / lower motor controller
     * @param topLimitSwitch The port of the upper limit switch on the climber
     * @param bottomLimitSwitch The port of the lower limit switch on the climber
     * @param driveMotorPort The ID of the motor controller used to move the wheel on the bottom of the climber
     */
    public DriveClimber(int motorPort, int topLimitSwitch, int bottomLimitSwitch, int driveMotorPort) {
        super(motorPort, topLimitSwitch, bottomLimitSwitch);

        m_driveMotor = new WPI_VictorSPX(driveMotorPort);
        m_driveMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void driveForward() {
        m_driveMotor.set(RobotMap.CLIMBER_DRIVE_SPEED_FOREWARD);
    }

    public void driveBackward() {
        m_driveMotor.set(RobotMap.CLIMBER_DRIVE_SPEED_BACKWARD);
    }

    public void driveRaw(double input) {
        m_driveMotor.set(input);
    }
}