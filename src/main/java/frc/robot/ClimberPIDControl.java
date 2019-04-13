package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;

/**
 * This class contains the config and the pid move method for the climbers
 * <p> 
 * Note that the PID config can only be used in sync, so we have to call climbers individually without a PID for retraction
 * @version Week 0 (Comp Season)
 */
public class ClimberPIDControl {
    Climber m_frontClimber;
	DriveClimber m_driveClimber;
	
	// Drivetrain needed exclusivley for its gyro
	NavX m_gyro;

	float initRoll;

	/**
	 * Constructor for the PID Control Configuration class for the climbing motor PIDs
	 * @param frontClimber The front climber instance
	 * @param backClimber The back climber instance
	 */
    public ClimberPIDControl(Climber frontClimber, DriveClimber backClimber, NavX gyro) {
        m_frontClimber = frontClimber;
		m_driveClimber = backClimber;
		m_gyro = gyro;
    }

    public void climberPIDConfig() {
		// Stops motor controllers
        m_frontClimber.m_climberMotor.set(ControlMode.PercentOutput, 0);
        m_driveClimber.m_climberMotor.set(ControlMode.PercentOutput, 0);

		// Set neutral mode
        m_frontClimber.m_climberMotor.setNeutralMode(NeutralMode.Brake);
        m_driveClimber.m_climberMotor.setNeutralMode(NeutralMode.Brake);

		/*
		// Configures sensor as quadrature encoder
		m_frontClimber.m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);
		
        // Configures the remote sensor for the drive talon
        m_driveClimber.m_climberMotor.configRemoteFeedbackFilter(m_frontClimber.m_climberMotor.getDeviceID(), RemoteSensorSource.TalonSRX_SelectedSensor, RobotMap.REMOTE_0, RobotMap.TIMEOUT_MS);

		// Adds the encoder values together and takes the average of them.
        m_driveClimber.m_climberMotor.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, RobotMap.TIMEOUT_MS);				// Feedback Device of Remote Talon
		m_driveClimber.m_climberMotor.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT_MS);	// Quadrature Encoder of current Talon
        m_driveClimber.m_climberMotor.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, RobotMap.TIMEOUT_MS);
		
		// Sets the encoder the primary climber PID listens to for input
        m_driveClimber.m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorSum, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);
        m_driveClimber.m_climberMotor.configSelectedFeedbackCoefficient(0.5, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);

		// Sets the encoder the "turning" climber PID listens to for input
        m_driveClimber.m_climberMotor.configSelectedFeedbackSensor(FeedbackDevice.SensorDifference, RobotMap.PID_TURN, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.configSelectedFeedbackCoefficient(1, RobotMap.PID_TURN, RobotMap.TIMEOUT_MS);
		
		// Zeroes the encoders on the climbers
		m_driveClimber.m_climberMotor.setSelectedSensorPosition(0, 0, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.setSelectedSensorPosition(0, 1, RobotMap.TIMEOUT_MS);
		m_frontClimber.m_climberMotor.setSelectedSensorPosition(0);
		*/

		// Config sensor and motor direction for the back climber motor
		m_driveClimber.m_climberMotor.setInverted(true);
		// m_driveClimber.m_climberMotor.setSensorPhase(true);

		// Config sensor and motor direction for the front climber motor
        m_frontClimber.m_climberMotor.setInverted(true);
		// m_frontClimber.m_climberMotor.setSensorPhase(true);

		/*
		// Set status frame period for data collection where 20 and 5 are length of period in ms
		m_driveClimber.m_climberMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.setStatusFramePeriod(StatusFrame.Status_10_Targets, 20, RobotMap.TIMEOUT_MS);
		m_frontClimber.m_climberMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.TIMEOUT_MS);
		*/
		// Config neutral deadband
        m_frontClimber.m_climberMotor.configNeutralDeadband(RobotMap.NEUTRAL_DEADBAND, RobotMap.TIMEOUT_MS);
        m_driveClimber.m_climberMotor.configNeutralDeadband(RobotMap.NEUTRAL_DEADBAND, RobotMap.TIMEOUT_MS);

		// Config peak output
		m_driveClimber.m_climberMotor.configPeakOutputForward(+1.0, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.configPeakOutputReverse(-1.0, RobotMap.TIMEOUT_MS);
        m_frontClimber.m_climberMotor.configPeakOutputForward(+1.0, RobotMap.TIMEOUT_MS);
		m_frontClimber.m_climberMotor.configPeakOutputReverse(-1.0, RobotMap.TIMEOUT_MS);

		/*
		// Motion Magic Config
		m_driveClimber.m_climberMotor.configMotionAcceleration(RobotMap.CLIMB_ACCELERATION, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.configMotionCruiseVelocity(RobotMap.CLIMB_CRUISE_VELOCITY, RobotMap.TIMEOUT_MS);

		// PID Config for the primary climber PID
		m_driveClimber.m_climberMotor.config_kP(0, RobotMap.CLIMBER_GAINS.kP, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.config_kI(0, RobotMap.CLIMBER_GAINS.kI, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.config_kD(0, RobotMap.CLIMBER_GAINS.kD, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.config_kF(0, RobotMap.CLIMBER_GAINS.kF, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.config_IntegralZone(0, RobotMap.CLIMBER_GAINS.kIzone, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.configClosedLoopPeakOutput(0, RobotMap.CLIMBER_GAINS.kPeakOutput, RobotMap.TIMEOUT_MS);
        m_driveClimber.m_climberMotor.configAllowableClosedloopError(0, 0, RobotMap.TIMEOUT_MS);
		
		// PID Config for the "turning" climber PID
        m_driveClimber.m_climberMotor.config_kP(1, RobotMap.GAINS_TURNING.kP, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.config_kI(1, RobotMap.GAINS_TURNING.kI, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.config_kD(1, RobotMap.GAINS_TURNING.kD, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.config_kF(1, RobotMap.GAINS_TURNING.kF, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.config_IntegralZone(1, RobotMap.GAINS_TURNING.kIzone, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.configClosedLoopPeakOutput(1, RobotMap.GAINS_TURNING.kPeakOutput, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.configAllowableClosedloopError(1, 0, RobotMap.TIMEOUT_MS);

		// PID closed loop config
        m_driveClimber.m_climberMotor.configClosedLoopPeriod(0, RobotMap.CLOSED_LOOP_TIME, RobotMap.TIMEOUT_MS);
        m_driveClimber.m_climberMotor.configClosedLoopPeriod(1, RobotMap.CLOSED_LOOP_TIME, RobotMap.TIMEOUT_MS);

		// Sets the frame period for how fast the PID runs to 10 ms
        m_driveClimber.m_climberMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, 10);

		// Sets profile slot for PID
        m_driveClimber.m_climberMotor.selectProfileSlot(0, RobotMap.PID_PRIMARY);
		m_driveClimber.m_climberMotor.selectProfileSlot(1, RobotMap.PID_TURN);
		*/

		initRoll = m_gyro.getRoll();
    }

	/**
	 * DO NOT USE UNTIL ENCODERS AND PID CONFIG IS RESTORED
	 * Sets the climbers to a set target for the PID to guide the climbers to go to
	 * @param target The target distance the PID goes to (in ticks)
	 */
    public void climberPIDDrive(int target) {
        m_driveClimber.m_climberMotor.set(ControlMode.MotionMagic, target, DemandType.AuxPID, 0);
        m_frontClimber.m_climberMotor.follow(m_driveClimber.m_climberMotor, FollowerType.AuxOutput1);
	}

	/**
	 * A bang bang drive method for lowering the climber to keep the climb even
	 * It defaults to the constants for speed down and then adjust based on difference from init yaw
	 */
	public void climberBangBang() {
		// A local variable for grabbing and storing roll temporarily
		float roll = m_gyro.getRoll();

		// Checks to see whether the difference is outside of the large threshold, and adjusts accordingly
		if (Math.abs(roll - initRoll) > RobotMap.BANG_BANG_DEADBAND_BIG) {
			// Checks which way the robot is tipping, where positive is tipping backwards
			if (roll > 0) {
				m_driveClimber.m_climberMotor.set(RobotMap.BACK_CLIMBER_SPEED_DOWN - RobotMap.BANG_BANG_ADJUST_LARGE);
				m_frontClimber.m_climberMotor.set(RobotMap.FRONT_CLIMBER_SPEED_DOWN + RobotMap.BANG_BANG_ADJUST_LARGE);
			}
			else {
				m_driveClimber.m_climberMotor.set(RobotMap.BACK_CLIMBER_SPEED_DOWN + RobotMap.BANG_BANG_ADJUST_LARGE);
				m_frontClimber.m_climberMotor.set(RobotMap.FRONT_CLIMBER_SPEED_DOWN - RobotMap.BANG_BANG_ADJUST_LARGE);
			}
		}
		// Checks to see whether the difference is outside of the small threshold, and adjusts accordingly
		else if (Math.abs(roll - initRoll) > RobotMap.BANG_BANG_DEADBAND_SMALL) {
			// Checks which way the robot is tipping, where positive is tipping backwards
			if (roll > 0) {
				m_driveClimber.m_climberMotor.set(RobotMap.BACK_CLIMBER_SPEED_DOWN - RobotMap.BANG_BANG_ADJUST_SMALL);
				m_frontClimber.m_climberMotor.set(RobotMap.FRONT_CLIMBER_SPEED_DOWN + RobotMap.BANG_BANG_ADJUST_SMALL);
			}
			else {
				m_driveClimber.m_climberMotor.set(RobotMap.BACK_CLIMBER_SPEED_DOWN + RobotMap.BANG_BANG_ADJUST_SMALL);
				m_frontClimber.m_climberMotor.set(RobotMap.FRONT_CLIMBER_SPEED_DOWN - RobotMap.BANG_BANG_ADJUST_SMALL);
			}
		}
		// Runs with the default values if within both thresholds
		else {
			m_driveClimber.m_climberMotor.set(RobotMap.BACK_CLIMBER_SPEED_DOWN);
			m_frontClimber.m_climberMotor.set(RobotMap.FRONT_CLIMBER_SPEED_DOWN);
		}
	}

}