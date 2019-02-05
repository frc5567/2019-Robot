package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DigitalInput;

public class Climber {

    private Spark m_climberMotor;
    private DigitalInput m_topLimitSwitch;

    /**
     * Constructor for the climber mechanism motor controllers and sensors.
     * @param motorPort Port of the motor controller
     * @param topLimitSwitchPort The port of the top limit switch on the climber
     */
    Climber(int motorPort, int topLimitSwitchPort) {
        m_climberMotor = new Spark(motorPort);
        m_topLimitSwitch = new DigitalInput(topLimitSwitchPort);
    }

    /**
     * Moves the climber based on the input recieved from controller
     * @param controllerInput Input from the controller to move climber (positive is up, negative is down)
     */
    public void moveClimber(double controllerInput) {
        double input = controllerInput;
        if ((input > 0) && m_topLimitSwitch.get()) {
            input = 0;
        }
        m_climberMotor.set(input);
    }
}