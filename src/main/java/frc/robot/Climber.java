package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrame;

public class Climber {

    private WPI_TalonSRX m_climberMotor;
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

    public void climberPIDConfig() {
		// Stops motor controllers
		m_climberMotor.set(ControlMode.PercentOutput, 0);

		// Set neutral mode
		m_climberMotor.setNeutralMode(NeutralMode.Brake);

		// Configures sensor as quadrature encoder
		m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);

		// Config sensor and motor direction
		m_climberMotor.setInverted(true);
		m_climberMotor.setSensorPhase(true);

		// Set status frame period for data collection
		m_climberMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.TIMEOUT_MS);

		// Config neutral deadband
		m_climberMotor.configNeutralDeadband(RobotMap.NEUTRAL_DEADBAND, RobotMap.TIMEOUT_MS);

		// Config peak output
		m_climberMotor.configPeakOutputForward(+RobotMap.PID_PEAK_OUTPUT, RobotMap.TIMEOUT_MS);
		m_climberMotor.configPeakOutputReverse(-RobotMap.PID_PEAK_OUTPUT, RobotMap.TIMEOUT_MS);

		// Motion Magic Config
		m_climberMotor.configMotionAcceleration(2000, RobotMap.TIMEOUT_MS);
		m_climberMotor.configMotionCruiseVelocity(2000, RobotMap.TIMEOUT_MS);

		// PID Config
		m_climberMotor.config_kP(0, RobotMap.GAINS.kP, RobotMap.TIMEOUT_MS);
		m_climberMotor.config_kI(0, RobotMap.GAINS.kI, RobotMap.TIMEOUT_MS);
		m_climberMotor.config_kD(0, RobotMap.GAINS.kD, RobotMap.TIMEOUT_MS);
		m_climberMotor.config_kF(0, RobotMap.GAINS.kF, RobotMap.TIMEOUT_MS);
		m_climberMotor.config_IntegralZone(0, RobotMap.GAINS.kIzone, RobotMap.TIMEOUT_MS);
		m_climberMotor.configClosedLoopPeakOutput(0, RobotMap.GAINS.kPeakOutput, RobotMap.TIMEOUT_MS);
		m_climberMotor.configAllowableClosedloopError(0, 0, RobotMap.TIMEOUT_MS);

		// PID closed loop config
		m_climberMotor.configClosedLoopPeriod(0, 5, RobotMap.TIMEOUT_MS);

		// Sets profile slot for PID
		m_climberMotor.selectProfileSlot(0, RobotMap.PID_PRIMARY);
    }

    public void climberPIDDrive(int target) {
		System.out.println("PIDTarget in tics: \t" + target);
		System.out.println("Current Position in tics: \t" + m_climberMotor.getSelectedSensorPosition());
		m_climberMotor.set(ControlMode.MotionMagic, target);
	}
    
    /**
     * Raises climber at a constant speed while button is pressed and the limit switch is not reached
     * (Button to be determined later).
     */
    public void raiseClimber(double speed) {
        if(!m_bottomLimitSwitch.get()) {
            m_climberMotor.set(speed);
        }
        else{
            m_climberMotor.set(0.0);
        }
    }

    /**
     * Lowers climber at a constant speed when button is pressed (Button to be determined later).
     */
    public void lowerClimber(double speed){
        if(!m_topLimitSwitch.get()){
            m_climberMotor.set(speed);
        }
        else{
            m_climberMotor.set(0.0);
        }
    }

    public void setClimber(double input) {
        m_climberMotor.set(ControlMode.PercentOutput, input);
    }

    /**
     * Used to get the status of the top break beam.
     * 
     * @return True if the top break beam is closed.
     */
    public boolean getTopLimitSwitch(){
        return m_topLimitSwitch.get();
    }

    /**
     * Used to get the status of the bottom break beam.
     * 
     * @return True if the bottim break beam is closed.
     */
    public boolean getBottomLimitSwitch(){
        return m_bottomLimitSwitch.get();
    }
}