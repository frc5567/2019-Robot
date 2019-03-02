package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

// this enum difines the buttons and what they do when active
public class GamePad extends GenericHID {

	/**
	 * Constructor, used for calling super constructor
	 * @param port Port the gamepad is connected to
	 */
	public GamePad(final int port) {
		super(port);
	}

	/**
	 * Actions each button performs
	 * Change numbers to correct port number
	 */
	private enum GamePadControls {
		// Buttons
		LOW_HATCH_CARGO(1),
		MEDIUM_HATCH_CARGO(2),
		HIGH_HATCH_CARGO(3),
		PICKUP_HATCH_CARGO(4),
		// Toggle switches
		HATCH_TO_CARGO(5),
		MANUAL_TO_AUTO(6),
		// Buttons
		OPEN_HATCH(7),
		CLOSE_HATCH(8),
		// Buttons
		LIFT_HATCH_ARM(9),
		DROP_HATCH_ARM(10),
		// Joystick X axis
		JOYSITCK_X_LEFT(11),
		JOYSTICK_X_RIGHT(12),
		// Joystick Y axis
		MANUAL_ELEVATOR_UP(13),
		MANUAL_ELEVATOR_DOWN(14);

		@SuppressWarnings("MemberName")
		public final int value;

		GamePadControls(int newValue) {
			this.value = newValue;
		}
	}

	/**
	 * Returns boolean stating if passed in game button is pressed
	 * @param button Button to view if pressed
	 * @return Returns button status (if pressed)
	 */
	public boolean getButtonPressed(GamePadControls button) {
		return super.getRawButtonPressed(button.value);
	}

	/**
	 * Returns if button has been released
	 * @param button Button to view if released
	 * @return Returns button status (if released)
	 */
	public boolean getGamePadButtonReleased(GamePadControls button) {
		return super.getRawButtonReleased(button.value);
	}

	/**
	 * Returns the status of the button (true if held)
	 * @param button Button to view if button is held
	 * @return Returns button status (if held)
	 */
	public boolean getGamePadButton(GamePadControls button) {
		return super.getRawButton(button.value);
	}

	/**
	 * Gets the X axis of the gamepad
	 * @return The value of the x axis
	 */
	public double getX(Hand hand) {
		System.out.println("Get X: \t" + getX());
		return getX();
	}

	/**
	 * Gets the Y axis of the gamepad
	 * @return The value of the y axis
	 */
	public double getY(Hand hand) {
		System.out.println("Get Y: \t" + getY());
		return getY();
	}

	/**
	 * Gets the status of the low hatch / cargo button
	 * @return If the low hatch / cargo button has been released
	 */
	public boolean getLowHatchCargoReleased() {
		return getGamePadButtonReleased(GamePadControls.LOW_HATCH_CARGO);
	}

	/**
	 * Gets the status of the medium hatch / cargo button
	 * @return If the medium hatch / cargo button has been released
	 */
	public boolean getMediumHatchCargoReleased() {
		return getGamePadButtonReleased(GamePadControls.MEDIUM_HATCH_CARGO);
	}

	/**
	 * Gets the status of the high hatch / cargo button
	 * @return If the high hatch / cargo button has been released
	 */
	public boolean getHighHatchCargoReleased() {
		return getGamePadButtonReleased(GamePadControls.HIGH_HATCH_CARGO);
	}

	/**
	 * Gets the status of the pickup hatch / cargo button
	 * @return If the pickup hatch / cargo button has been released
	 */
	public boolean getPickupHatchCargoReleased() {
		return getGamePadButtonReleased(GamePadControls.PICKUP_HATCH_CARGO);
	}

	/**
	 * Gets the status of the hatch to cargo button
	 * @return If the hatch to cargo button has been released
	 */
	public boolean getHatchToCargoReleased() {
		return getGamePadButtonReleased(GamePadControls.HATCH_TO_CARGO);
	}

	/**
	 * Gets the status of the manual to auto button
	 * @return If the manual to auto button has been released
	 */
	public boolean getManualToAutoReleased() {
		return getGamePadButtonReleased(GamePadControls.MANUAL_TO_AUTO);
	}

	/**
	 * Gets the status of the open hatch button
	 * @return If the open hatch button has been released
	 */
	public boolean getOpenHatchReleased() {
		return getGamePadButtonReleased(GamePadControls.OPEN_HATCH);
	}

	/**
	 * Gets the status of the close hatch button
	 * @return If the close hatch button has been released
	 */
	public boolean getCloseHatchReleased() {
		return getGamePadButtonReleased(GamePadControls.CLOSE_HATCH);
	}

	/**
	 * Gets the status of the hatch arm lift button
	 * @return If the hatch arm lift button has been released
	 */
	public boolean getLiftHatchArmReleased() {
		return getGamePadButtonReleased(GamePadControls.LIFT_HATCH_ARM);
	}

	/**
	 * Gets the status of the hatch arm drop button
	 * @return If the hatch arm drop button has been released
	 */
	public boolean getDropHatchArmReleased() {
		return getGamePadButtonReleased(GamePadControls.DROP_HATCH_ARM);
	}

	/**
	 * Gets the current state of the pickup hatch button
	 * @return Returns the current status of the button (held)
	 */
	public boolean getPickupHatchCargo() {
		return getGamePadButton(GamePadControls.PICKUP_HATCH_CARGO);
	}

	/**
	 * Gets the current state of the low hatch button
	 * @return Returns the current status of the button (held)
	 */
	public boolean getLowHatchCargo() {
		return getGamePadButton(GamePadControls.LOW_HATCH_CARGO);
	}

	/**
	 * Gets the current state of the medium hatch button
	 * @return Returns the current status of the button (held)
	 */
	public boolean getMediumHatchCargo() {
		return getGamePadButton(GamePadControls.MEDIUM_HATCH_CARGO);
	}

	/**
	 * Gets the current state of the high hatch button
	 * @return Returns the current status of the button (held)
	 */
	public boolean getHighHatchCargo() {
		return getGamePadButton(GamePadControls.HIGH_HATCH_CARGO);
	}

	/**
	 * Gets the current state of the lift hatch arm button
	 * @return Returns the current status of the button (held)
	 */
	public boolean getLiftHatchArm() {
		return getGamePadButton(GamePadControls.LIFT_HATCH_ARM);
	}

	/**
	 * Gets the current state of the drop hatch arm button
	 * @return Returns the current status of the button (held)
	 */
	public boolean getDropHatchArm() {
		return getGamePadButton(GamePadControls.DROP_HATCH_ARM);
	}

	public boolean getJoystickXLeft() {
		return getGamePadButton(GamePadControls.JOYSITCK_X_LEFT);
	}

	public boolean getJoystickXRight() {
		return getGamePadButton(GamePadControls.JOYSTICK_X_RIGHT);
	}

	public boolean getManualElevatorUp() {
		return getGamePadButton(GamePadControls.MANUAL_ELEVATOR_UP);
	}

	public boolean getManulaElevatorDown() {
		return getGamePadButton(GamePadControls.MANUAL_ELEVATOR_DOWN);
	}
}