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
		LOW_HATCH_CARGO(1),
		MEDIUM_HATCH_CARGO(2),
		HIGH_HATCH_CARGO(3),
		PICKUP_HATCH_CARGO(4),
		HATCH_TO_CARGO(5),
		MANUAL_TO_AUTO(6),
		OPEN_HATCH(7),
		CLOSE_HATCH(8),
		LIFT_HATCH_ARM(9),
		DROP_HATCH_ARM(10);

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
	public boolean getGPButtonPressed(GamePadControls button) {
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
	 * Gets the X axis of the gamepad
	 * @return The value of the x axis
	 */
	public double getX(Hand hand) {
		return Double.NaN;
	}

	/**
	 * Gets the Y axis of the gamepad
	 * @return The value of the y axis
	 */
	public double getY(Hand hand) {
		return Double.NaN;
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
}
