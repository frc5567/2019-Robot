package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climber {

    private WPI_VictorSPX m_climberMotor;
    private DigitalInput m_topLimitSwitch;
    private DigitalInput m_bottomLimitSwitch;

    /**
     * Constructor for the climber mechanism motor controllers and sensors.
     * 
     * @param motorPort          Port of the motor controller
     * @param topLimitSwitchPort The port of the top limit switch on the climber
     * @param bottomLimitSwitchPort The port of the bottom limit switch on the climber
     */
    public Climber(int motorPort, int topLimitSwitchPort, int bottomLimitSwitchPort) {
        m_climberMotor = new WPI_VictorSPX(motorPort);
        m_topLimitSwitch = new DigitalInput(topLimitSwitchPort);
        m_bottomLimitSwitch = new DigitalInput(bottomLimitSwitchPort);
    }

    /**
     * Raises climber at a constant speed while button is pressed and the limit switch is not reached
     * (Button to be determined later).
     */
    public void raiseClimber() {
        if(!m_bottomLimitSwitch.get()) {
            m_climberMotor.set(RobotMap.CLIMBER_SPEED_UP);
        }
        else{
            m_climberMotor.set(0.0);
        }
    }

    /**
     * Lowers climber at a constant speed when button is pressed (Button to be determined later).
     */
    public void lowerClimber(){
        if(!m_topLimitSwitch.get()){
            m_climberMotor.set(RobotMap.CLIMBER_SPEED_DOWN);
        }
        else{
            m_climberMotor.set(0.0);
        }
    }

    public void setClimber(double input) {
        m_climberMotor.set(input);
    }
}