package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climber {

    private Spark m_climberMotor;
    private DigitalInput m_topLimitSwitch;

    /**
     * Constructor for the climber mechanism motor controllers and sensors.
     * 
     * @param motorPort          Port of the motor controller
     * @param topLimitSwitchPort The port of the top limit switch on the climber
     */
    public Climber(int motorPort, int topLimitSwitchPort) {
        m_climberMotor = new Spark(motorPort);
        m_topLimitSwitch = new DigitalInput(topLimitSwitchPort);
    }

    /**
     * Raises climber at a constant speed while button is pressed and the limit switch is not reached
     * (Button to be determined later).
     * @param buttonInput Input from the controller button of our choice.
     */
    public void raiseClimber(boolean buttonInput) {
        if(buttonInput && !m_topLimitSwitch.get()) {
            m_climberMotor.set(RobotMap.CLIMBER_SPEED_UP);
        }
        else{
            m_climberMotor.set(0.0);
        }
        return;
    }

    /**
     * Lowers climber at a constant speed when button is pressed (Button to be determined later).
     * 
     * @param buttonInput Input from the controller based on the button of our choice.
     */
    public void lowerClimber(boolean buttonInput){
        if(buttonInput){
            m_climberMotor.set(RobotMap.CLIMBER_SPEED_DOWN);
        }
        else{
            m_climberMotor.set(0.0);
        }
        return;
    }
}