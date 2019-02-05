package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * This class defines the mechanism that moves up and down for hatch covers and
 * cargo.
 */
public class Elevator {

	public enum State {
		LEVEL_ZERO(0.0, 1.0, 1.0),
		CARGO_L1(16.75, 0.80, 0.50),
		CARGO_L2(44.75, 0.60, 0.30),
		CARGO_L3(72.75, 0.40, 0.20),
		HATCH_L1(8.25, 0.90, 0.50),
		HATCH_L2(36.25, 0.65, 0.30),
		HATCH_L3(64.25, 0.45, 0.20);

		private double deltaHeightInches;
		private double maxSpeedPercent;
		private double maxAngleRate;

		/**
		 * 
		 * 
		 * @param deltaInches      Change in height from position zero in inches.
		 * @param maxSpeedModifier Percent of our max speed we use when the elevator is
		 *                         in this state.
		 * @param maxAngleRate     How fast our angle motor can move when the elevator
		 *                         is in this state.
		 */
		State(double deltaInches, double maxSpeedModifier, double maxAngleRate) {
			this.deltaHeightInches = deltaInches;
			this.maxSpeedPercent = maxSpeedModifier;
			this.maxAngleRate = maxAngleRate;
		}

		/**
		 * @return The change in height from initial to current state.
		 */
		public double getDeltaHeight() {
			return this.deltaHeightInches;
		}

		/**
		 * @return The max speed modifier based on the elevator's current state.
		 */
		public double maxSpeedModifier() {
			return this.maxSpeedPercent;
		}

		/**
		 * @return The max speed we turn our angle arm at the elevator's current state.
		 */
		public double maxAngleRate() {
			return this.maxAngleRate;
		}
	}

	// Defining the limit switches at the top and bottom of the elevator.
	DigitalInput m_limitTop;
	DigitalInput m_limitBottom;

	// Declaring the encoder for the elevator height.
	SensorCollection m_elevatorEncoder;

	// Declaring the speed controller for the elevator.
	WPI_TalonSRX m_elevatorMotor;

	// Declaring the elevator state enum.
	State currentState;

	// This constructor is initializing in creating a new instance of an elevator
	// with limit port switch definitions.

	Elevator(double distancePerPulse) {

		// Instantiates Motor controller for elevator
		m_elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_PORT);

		// Creating a new instance of DigitalInput with the assigned port number.
		m_limitTop = new DigitalInput(RobotMap.ELEVATOR_LIMIT_TOP_PORT);
		m_limitBottom = new DigitalInput(RobotMap.ELEVATOR_LIMIT_BOTTOM_PORT);

		// Instantiating encoder for the elevator height
		m_elevatorEncoder = new SensorCollection(m_elevatorMotor);

		// Zeroes the encoder
		m_elevatorEncoder.setQuadraturePosition(0, 0);
	}

	/**
	 * Returns the current position of the elevator reported by the encoder
	 * 
	 * @return The position of the elevator encoder
	 */
	public int getElevatorEncoderPosition() {
		return m_elevatorEncoder.getQuadraturePosition();
	}

	/**
	 * Returns the current velocity of the elevator reported by the elevator encoder
	 * 
	 * @return The velocity of the elevator encoder
	 */
	public int getElevatorEncoderVelocity() {
		return m_elevatorEncoder.getQuadratureVelocity();
	}
}