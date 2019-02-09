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
    Climber(int motorPort, int topLimitSwitchPort) {
        m_climberMotor = new Spark(motorPort);
        m_topLimitSwitch = new DigitalInput(topLimitSwitchPort);
    }

    /**
     * Raises climber when button is pressed
     * 
     * @param buttonInput Input from the controller button of our choice.
     */
    public void raiseClimber(boolean buttonInput) {
        if(buttonInput){
            double input;
            if (m_topLimitSwitch.get()){
                input = 0;
            } 
            else {
                input = RobotMap.CLIMBER_SPEED_UP;
            }
            m_climberMotor.set(input);
        }
        else{
            return;
        }
    }

    /**
     * Lowers climber when button is pressed
     * 
     * @param buttonInput Input from the controller based on the button of our choice.
     */
    public void lowerClimber(boolean buttonInput){
        if(buttonInput){
            m_climberMotor.set(RobotMap.CLIMBER_SPEED_DOWN);
        }
        else{
            return;
        }
    }
}