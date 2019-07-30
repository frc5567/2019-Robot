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

		// Config sensor and motor direction for the back climber motor
		m_driveClimber.m_climberMotor.setInverted(true);

		// Config sensor and motor direction for the front climber motor
        m_frontClimber.m_climberMotor.setInverted(true);

		// Config neutral deadband
        m_frontClimber.m_climberMotor.configNeutralDeadband(RobotMap.NEUTRAL_DEADBAND, RobotMap.TIMEOUT_MS);
        m_driveClimber.m_climberMotor.configNeutralDeadband(RobotMap.NEUTRAL_DEADBAND, RobotMap.TIMEOUT_MS);

		// Config peak output
		m_driveClimber.m_climberMotor.configPeakOutputForward(+1.0, RobotMap.TIMEOUT_MS);
		m_driveClimber.m_climberMotor.configPeakOutputReverse(-1.0, RobotMap.TIMEOUT_MS);
        m_frontClimber.m_climberMotor.configPeakOutputForward(+1.0, RobotMap.TIMEOUT_MS);
		m_frontClimber.m_climberMotor.configPeakOutputReverse(-1.0, RobotMap.TIMEOUT_MS);

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