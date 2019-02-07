package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;


// this enum difines the buttons and what they do when active
public class GamePad extends GenericHID 
{

  public GamePad(final int port) {
    super(port);
  }

  //these are the actions that each button preforms
  // change the numbers to the correct port number
  private enum GamePadControls {
    LOW_HATCH_CARGO (1),
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
 
  //this will tell us when the button is pressed
  public boolean getGPButtonPressed(GamePadControls button) {
    return super.getRawButtonPressed(button.value);
  }

  // this will tell us when the button is released
  public boolean getGamePadButtonReleased(GamePadControls button) {
    return super.getRawButtonReleased(button.value);
  }

  public double getX(Hand hand) {
    return Double.NaN;
  }
  
  public double getY(Hand hand){
    return Double.NaN;
  }
}