package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * This class is an extended class for the XboxController class WPI provided
 * This adds methods with deadbands on the analog sticks on triggers
 * @author Matt
 * @version Week 5 Pre-comp
 */
public class Controller extends XboxController {

    /**
     * Handles for input from driver's Xbox Controllers, and abjusts analog input using deadbands
     * @param port The Driver Station USB the controller is plugged into
     */
    public Controller(int port) {
        super(port);
    }

    /**
     * Gets the value of the left stick X axis
     * @return The X value of the left stick
     */
    public double getX(Hand hand) {
        double stickXValue;
        // Checks if stick is within deadband
        if ((super.getX(hand) < RobotMap.CONTROLLER_STICK_DEADBAND) && (super.getX(hand) > -RobotMap.CONTROLLER_STICK_DEADBAND)) {
            // If so, stick value is set to 0
            stickXValue = 0.00;
        }
        // If the stick is outside deadband, sets variable to stick value
        else {
            stickXValue = super.getX(hand);
        }
        // Returns the stick value
        return stickXValue;
    }

    /**
     * Gets the value of the left stick Y axis
     * @return The Y value of the left axis
     */
    public double getY(Hand hand) {
        double stickYValue;
        // Checks if stick value is within deadband
        if ((super.getY(hand) < RobotMap.CONTROLLER_STICK_DEADBAND) && (super.getY(hand) > -RobotMap.CONTROLLER_STICK_DEADBAND)) {
            // If so, stick value is set to 0
            stickYValue = 0.00;
        }
        // If the stick is outside deadband, sets variable to stick value
        else {
            // Negative added to fix y-stick values
            // Up on stick is now positive
            stickYValue = -super.getY(hand);
        }
        // Returns the stick value
        return stickYValue;
    }

    /**
     * Gets the value of the left trigger
     * @return The value of the left trigger axis
     */
    public double getTriggerAxis(Hand hand) {
        double triggerValue;
        // Checks if the trigger value is less than deadband
        // if (super.getTriggerAxis(hand) < RobotMap.CONTROLLER_TRIGGER_DEADBAND) {
        //     // If so, sets the trigger value to 0
        //     triggerValue = 0.00;
        // }
        // If the trigger is outside of deadband, sets the variable to trigger value
        // else {
            triggerValue = super.getTriggerAxis(hand);
        // }
        // Returns trigger value
        return triggerValue;
    }
}