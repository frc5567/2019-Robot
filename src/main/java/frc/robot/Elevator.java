package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/* TODO 
Add method for setting elevator State using encoder position once we get proper
values from the encoders. 
*/

/**
 * This class defines the mechanism that moves up and down for hatch covers and
 * cargo.
 */
public class Elevator {
	
	/**
	 * Enum used to contain values related to the position of the elevator.
	 */
	public enum State {
		/*
		From left to right: Change in height from position zero, modification to our max speed when
		in that state, the max speed we angle the placer when in that state, and the name of the
		state in String form.
		*/
		LEVEL_ZERO(0.0, 1.0, 1.0 , "Initial State"),
		CARGO_L1(16.75, 0.80, 0.50 , "Cargo level 1"),
		CARGO_L2(44.75, 0.60, 0.30 , "Cargo Level 2"),
		CARGO_L3(72.75, 0.40, 0.20 , "Cargo Level 3"),
		HATCH_L1(8.25, 0.90, 0.50 , "Hatch Level 1"),
		HATCH_L2(36.25, 0.65, 0.30 , "Hatch Level 2"),
		HATCH_L3(64.25, 0.45, 0.20 , "Hatch Level 3"),
		TRANSITION(0.0, 0.40, 0.20, "Transitioning");

		private double deltaInches;
		private double maxSpeedPercent;
		private double maxAngleRate;
		private String stateName;

		/**
		 * 
		 * 
		 * @param deltaInches      Change in height from position zero in inches.
		 * @param maxSpeedModifier Percent of our max speed we use when the elevator is
		 *                         in this state.
		 * @param stateName A string representation of the state we are in.
		 * @param maxAngleRate     How fast our angle motor can move when the elevator
		 *                         is in this state.
		 */
		State(double deltaInches, double maxSpeedModifier, double maxAngleRate , String stateName) {
			this.deltaInches = deltaInches;
			this.maxSpeedPercent = maxSpeedModifier;
			this.maxAngleRate = maxAngleRate;
			this.stateName = stateName;
		}

		/**
		 * @return The change in height from initial to current state.
		 */
		public double getDeltaHeight() {
			return this.deltaInches;
		}

		/**
		 * @return The max speed modifier based on the elevator's current state.
		 */
		public double getMaxSpeedModifier() {
			return this.maxSpeedPercent;
		}

		/**
		 * @return The max speed we turn our angle arm at the elevator's current state.
		 */
		public double getMaxAngleRate() {
			return this.maxAngleRate;
		}

		/**
		 * @return The name of the state we are currently in.
		 */
		public String getStateName(){
			return this.stateName;
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

	public Elevator() {

		// Instantiates Motor controller for elevator
		m_elevatorMotor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_PORT);

		// Creating a new instance of DigitalInput with the assigned port number.
		m_limitTop = new DigitalInput(RobotMap.ELEVATOR_LIMIT_TOP_PORT);
		m_limitBottom = new DigitalInput(RobotMap.ELEVATOR_LIMIT_BOTTOM_PORT);

		// Instantiating encoder for the elevator height
		m_elevatorEncoder = new SensorCollection(m_elevatorMotor);

		// Zeroes the encoder
		m_elevatorEncoder.setQuadraturePosition(0, 0);

		// Sets the State enum to it's initial state
		currentState = State.LEVEL_ZERO;
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
	
	/**
	 * Calculates and returns the height of the elevator in inches.
	 * 
	 * @return The elevator's current height
	 */
	public double getPosition(){
		double position = 0.0;
		double numRevolutions = (m_elevatorEncoder.getQuadraturePosition() / RobotMap.TICKS_PER_REVOLUTION);
		position = RobotMap.DRUM_CIRCUMFERENCE * numRevolutions;
		currentState = getState(position);
		return position;
	}

	/**
	 * Method to set currentState based on the current height of the elevator.
	 * 
	 * @param position The position gathered by the getPosition method.
	 * @return The state of the elevator based on it's height.
	 */
	private State getState(double position){
		if(position < State.CARGO_L1.deltaInches + 2 && position > State.CARGO_L1.deltaInches - 2){
			return State.CARGO_L1;
		}
		else if(position < State.CARGO_L2.deltaInches + 2 && position > State.CARGO_L2.deltaInches - 2){
			return State.CARGO_L2;
		}
		else if(position < State.CARGO_L3.deltaInches + 2 && position > State.CARGO_L3.deltaInches - 2){
			return State.CARGO_L3;
		}
		else if(position < State.HATCH_L1.deltaInches + 2 && position > State.HATCH_L1.deltaInches - 2){
			return State.HATCH_L1;
		}
		else if(position < State.HATCH_L2.deltaInches + 2 && position > State.HATCH_L2.deltaInches - 2){
			return State.HATCH_L2;
		}
		else if(position < State.HATCH_L3.deltaInches + 2 && position > State.HATCH_L3.deltaInches - 2){
			return State.HATCH_L3;
		}
		else{
			return State.TRANSITION;
		}
	}

}