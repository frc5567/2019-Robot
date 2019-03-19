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
     * @param topLimitSwitchPort The port of the top limit switch on the climber
     * @param bottomLimitSwitchPort The port of the bottom limit switch on the climber
     */
    public Climber(int motorPort, int topLimitSwitchPort, int bottomLimitSwitchPort) {
        m_climberMotor = new WPI_TalonSRX(motorPort);
        m_topLimitSwitch = new DigitalInput(topLimitSwitchPort);
        m_bottomLimitSwitch = new DigitalInput(bottomLimitSwitchPort);
    }
    
    /**
     * Raises climber at a constant speed while button is pressed and the limit switch is not reached
     * (Button to be determined later).
     */
    public void raiseClimber(double speed) {
        m_climberMotor.set(speed);
    }

    /**
     * Lowers climber at a constant speed when button is pressed (Button to be determined later).
     */
    public void lowerClimber(double speed){
        m_climberMotor.set(speed);
    }

    public void setClimber(double input) {
        m_climberMotor.set(ControlMode.PercentOutput, input);
    }

    /**
     * Used to get the status of the top break beam.
     * Limit Switch not mounted, this should not be used or called
     * 
     * @return True if the top break beam is closed.
     */
    public boolean getTopLimitSwitch(){
        return m_topLimitSwitch.get();
    }

    /**
     * Used to get the status of the bottom break beam.
     * Limit Switch not mounted, this should not be used or called
     * 
     * @return True if the bottim break beam is closed.
     */
    public boolean getBottomLimitSwitch(){
        return m_bottomLimitSwitch.get();
    }
}