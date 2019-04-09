package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * This class defines the mechanism that moves up and down for hatch covers.
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
		HATCH_PICKUP(8.0, 0.90, 0.50, "Hatch Pickup"),
		//	CARGO_L1(16.75, 0.80, 0.50 , "Cargo level 1"),
		//	CARGO_L2(44.75, 0.60, 0.30 , "Cargo Level 2"),
		//	CARGO_L3(72.75, 0.40, 0.20 , "Cargo Level 3"),
		HATCH_L1(9.0, 0.90, 0.50 , "Hatch Level 1"),
		HATCH_L2(40.65, 0.65, 0.30 , "Hatch Level 2"),
		HATCH_L3(66.125, 0.45, 0.20 , "Hatch Level 3"),
		HATCH_PICKUP_2(11.87, 0.90, 0.50, "Hatch Pickup 2");

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

	// Declaring the encoder for the elevator height.
	SensorCollection m_encoder;

	// Declaring the speed controller for the elevator.
	WPI_TalonSRX m_motor;

	// Declaring the elevator state enum.
	State currentState;

	// Tracking Variables for Motion Magic
	boolean m_firstCall;
	double m_lockedDistance;
	double m_targetAngle;
	int m_smoothing;

	// This constructor is initializing in creating a new instance of an elevator
	// with limit port switch definitions.

	public Elevator() {

		// Instantiates Motor controller for elevator
		m_motor = new WPI_TalonSRX(RobotMap.ELEVATOR_MOTOR_PORT);

		// Instantiating encoder for the elevator height
		m_encoder = new SensorCollection(m_motor);

		// Zeroes the encoder
		m_encoder.setQuadraturePosition(0, 0);

		// Sets the State enum to it's initial state
		currentState = State.LEVEL_ZERO;

		m_firstCall = false;
		m_lockedDistance = 0;
		m_targetAngle = 0;
	}

	/**
	 * Configures the elevator's PID controller.
	 */
	public void configPID() {
		// Stops motor controllers
		m_motor.set(ControlMode.PercentOutput, 0);

		// Set neutral mode
		m_motor.setNeutralMode(NeutralMode.Brake);

		// Configures sensor as quadrature encoder
		m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.PID_PRIMARY, RobotMap.TIMEOUT_MS);

		// Config sensor and motor direction
		m_motor.setInverted(true);
		m_motor.setSensorPhase(true);

		// Set status frame period for data collection where 5 is period length in ms
		m_motor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.TIMEOUT_MS);

		// Config neutral deadband
		m_motor.configNeutralDeadband(RobotMap.NEUTRAL_DEADBAND, RobotMap.TIMEOUT_MS);

		// Config peak output
		m_motor.configPeakOutputForward(+RobotMap.PID_PEAK_OUTPUT, RobotMap.TIMEOUT_MS);
		m_motor.configPeakOutputReverse(-RobotMap.PID_PEAK_OUTPUT, RobotMap.TIMEOUT_MS);

		// Motion Magic Config
		m_motor.configMotionAcceleration(RobotMap.ELEVATOR_ACCELERATION, RobotMap.TIMEOUT_MS);
		m_motor.configMotionCruiseVelocity(RobotMap.ELEVATOR_CRUISE_VELOCITY, RobotMap.TIMEOUT_MS);

		// PID Config
		m_motor.config_kP(0, RobotMap.ELEVATOR_GAINS.kP, RobotMap.TIMEOUT_MS);
		m_motor.config_kI(0, RobotMap.ELEVATOR_GAINS.kI, RobotMap.TIMEOUT_MS);
		m_motor.config_kD(0, RobotMap.ELEVATOR_GAINS.kD, RobotMap.TIMEOUT_MS);
		m_motor.config_kF(0, RobotMap.ELEVATOR_GAINS.kF, RobotMap.TIMEOUT_MS);
		m_motor.config_IntegralZone(0, RobotMap.ELEVATOR_GAINS.kIzone, RobotMap.TIMEOUT_MS);
		m_motor.configClosedLoopPeakOutput(0, RobotMap.ELEVATOR_GAINS.kPeakOutput, RobotMap.TIMEOUT_MS);
		m_motor.configAllowableClosedloopError(0, 0, RobotMap.TIMEOUT_MS);

		// PID closed loop config
		m_motor.configClosedLoopPeriod(0, 10, RobotMap.TIMEOUT_MS);

		// Sets profile slot for PID
		m_motor.selectProfileSlot(0, RobotMap.PID_PRIMARY);
	}

	/**
	 * Moves the elevator to a specified position based on which State is passed in.
	 * @param state The desired state of the elevator (see State enum)
	 * @return False if the encoder velocity remains below 100 ticks per millisecond.
	 */
	public boolean drivePID(State state) {
		double target = (state.deltaInches) * (RobotMap.TICKS_PER_REVOLUTION / RobotMap.DRUM_CIRCUMFERENCE);
		m_motor.set(ControlMode.MotionMagic, target);

		if(getEncoderVelocity() > 100){
			return false;
		}
		else{
			return true;
		}
	}

	/**
	 * Returns the current position of the elevator reported by the encoder
	 * 
	 * @return The position of the elevator encoder
	 */
	public int getEncoderPosition() {
		return m_encoder.getQuadraturePosition();
	}

	/**
	 * Returns the current velocity of the elevator reported by the elevator encoder
	 * 
	 * @return The velocity of the elevator encoder
	 */
	public int getEncoderVelocity() {
		return m_encoder.getQuadratureVelocity();
	}

	/**
	 * Method used to manually move the elevator.
	 * @param input Joystick/variable input.
	 */
	public void moveRaw(double input){
			m_motor.set(ControlMode.PercentOutput, (input));
	}
	
	/**
	 * Calculates and returns the height of the elevator in inches.
	 * 
	 * @return The elevator's current height
	 */
	public double getPosition(){
		double positionInches = (m_motor.getSelectedSensorPosition() * (RobotMap.DRUM_CIRCUMFERENCE / RobotMap.TICKS_PER_REVOLUTION));
		//position = RobotMap.DRUM_CIRCUMFERENCE * numRevolutions;
		currentState = getState(positionInches);
		return positionInches;
	}

	/**
	 * Method used to move the elevator to our desired height while a button of our choice
	 * is pressed.
	 * 
	 * @param button The button we want associated with moving to the desired state.
	 * @param desiredState The state we want the button to move the elevator to.
	 */
	public void moveToPosition(boolean button , State desiredState){
		if(button && currentState != desiredState){
			if(currentState.deltaInches > desiredState.deltaInches){
				m_motor.set(RobotMap.ELEVATOR_MOTOR_SPEED_DOWN);
			}
			else if(currentState.deltaInches < desiredState.deltaInches){
				m_motor.set(RobotMap.ELEVATOR_MOTOR_SPEED_UP);
			}
		}
		else if(currentState == desiredState){
			m_motor.set(0.0);
		}
	}

	/**
	 * Method to set currentState based on the current height of the elevator.
	 * 
	 * @param position The position gathered by the getPosition method.
	 * @return The state of the elevator based on it's height.
	 */
	private State getState(double position){
		// if(position < State.CARGO_L1.deltaInches + 0.7 && position > State.CARGO_L1.deltaInches - 0.7){
		// 	return State.CARGO_L1;
		// }
		// else if(position < State.CARGO_L2.deltaInches + 0.7 && position > State.CARGO_L2.deltaInches - 0.7){
		// 	return State.CARGO_L2;
		// }
		// else if(position < State.CARGO_L3.deltaInches + 0.7 && position > State.CARGO_L3.deltaInches - 0.7){
		// 	return State.CARGO_L3;
		// }
		if(position < State.HATCH_L1.deltaInches + 0.7 && position > State.HATCH_L1.deltaInches - 0.7){
			return State.HATCH_L1;
		}
		else if(position < State.HATCH_L2.deltaInches + 0.7 && position > State.HATCH_L2.deltaInches - 0.7){
			return State.HATCH_L2;
		}
		else if(position < State.HATCH_L3.deltaInches + 1.5 && position > State.HATCH_L3.deltaInches - 1.5){
			return State.HATCH_L3;
		}
		else if(position < State.HATCH_L1.deltaInches - 0.7) {
			return State.LEVEL_ZERO;
		}
		else{
			return currentState;
		}
	}

}