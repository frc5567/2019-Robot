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
		LOW_HATCH_CARGO(6),
		MEDIUM_HATCH_CARGO(7),
		HIGH_HATCH_CARGO(8),
		PICKUP_HATCH_CARGO(5),
		// Toggle switches
		HATCH_TO_CARGO(10),
		MANUAL_SWITCH(9),
		// Buttons
		OPEN_HATCH(2),
		CLOSE_HATCH(1),
		// Buttons
		LIFT_HATCH_ARM(4),
		DROP_HATCH_ARM(3),
		PICKUP_1(12),
		ZERO_HATCH(11);

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
		return getX();
	}

	/**
	 * Gets the Y axis of the gamepad
	 * @return The value of the y axis
	 */
	public double getY(Hand hand) {
		return getY();
	}

	/**
	 * Gets the status of the level zero button
	 * @return If the level zero button is being pressed
	 */
	public boolean getLevelZero() {
		return getGamePadButton(GamePadControls.ZERO_HATCH);
	}

	public boolean getLevelZeroReleased() {
		return getGamePadButtonReleased(GamePadControls.ZERO_HATCH);   
	}

	/**
	 * Gets the status of the low pickup button
	 * @return If the low pickup button is being pressed
	 */
	public boolean getPickup1Button() {
		return getGamePadButton(GamePadControls.PICKUP_1);
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
	 * Gets the current state of the manual or auto mode switch, where true is manual
	 * @return Whether the robot is in manual mode
	*/ 
	public boolean isManual() {
		return getGamePadButton(GamePadControls.MANUAL_SWITCH);
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
}