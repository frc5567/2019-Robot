package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climber {

    WPI_TalonSRX m_climberMotor;
    private DigitalInput m_topLimitSwitch;
    private DigitalInput m_bottomLimitSwitch;

    /**
     * Constructor for the climber mechanism motor controllers and sensors.
     * 
     * @param motorPort          Port of the motor controller
     */
    public Climber(int motorPort) {
        m_climberMotor = new WPI_TalonSRX(motorPort);
    }

    /**
     * Manually sets the climber to a passed in speed [-1.0 ~ 1.0]
     * @param speed Percent output to set the climber motor to
     */
    public void setClimberSpeed(double speed) {
        m_climberMotor.set(ControlMode.PercentOutput, speed);
    }
}