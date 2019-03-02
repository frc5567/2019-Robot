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
    public double getLeftStickX() {
        double leftStickXValue;
        // Checks if stick is within deadband
        if ((m_myController.getX(Hand.kLeft) < RobotMap.CONTROLLER_STICK_DEADBAND) && (m_myController.getX(Hand.kLeft) > -RobotMap.CONTROLLER_STICK_DEADBAND)) {
            // If so, stick value is set to 0
            leftStickXValue = 0.00;
        }
        // If the stick is outside deadband, sets variable to stick value
        else {
            leftStickXValue = m_myController.getX(Hand.kLeft);
        }
        // Returns the stick value
        return leftStickXValue;
    }

    /**
     * Gets the value of the left stick Y axis
     * @return The Y value of the left axis
     */
    public double getLeftStickY() {
        double leftStickYValue;
        // Checks if stick value is within deadband
        if ((m_myController.getY(Hand.kLeft) < RobotMap.CONTROLLER_STICK_DEADBAND) && (m_myController.getY(Hand.kLeft) > -RobotMap.CONTROLLER_STICK_DEADBAND)) {
            // If so, stick value is set to 0
            leftStickYValue = 0.00;
        }
        // If the stick is outside deadband, sets variable to stick value
        else {
            // Negative added to fix y-stick values
            // Up on stick is now positive
            leftStickYValue = -m_myController.getY(Hand.kLeft);
        }
        // Returns the stick value
        return leftStickYValue;
    }

    /**
     * Gets the value of the right stick X axis
     * @return The X value of the right axis
     */
    public double getRightStickX() {
        double rightStickXValue;
        // Checks if the stick value is within deadband
        if ((m_myController.getX(Hand.kRight) < RobotMap.CONTROLLER_STICK_DEADBAND) && (m_myController.getX(Hand.kRight) > -RobotMap.CONTROLLER_STICK_DEADBAND)) {
            // If so, set stick value to 0
            rightStickXValue = 0.00;
        }
        // If the stick is outside deadband, sets variable to stick value
        else {
            rightStickXValue = m_myController.getX(Hand.kRight);
        }
        // Returns the stick value
        return rightStickXValue;
    }

    /**
     * Gets the value of the right stick Y axis
     * @return The Y value of the right axis
     */
    public double getRightStickY() {
        double rightStickYValue;
        // Checks if the stick value is within deadband
        if ((m_myController.getY(Hand.kRight) < RobotMap.CONTROLLER_STICK_DEADBAND) && (m_myController.getY(Hand.kRight) > -RobotMap.CONTROLLER_STICK_DEADBAND)) {
            // If so, set stick value to 0
            rightStickYValue = 0.00;
        }
        // If the stick is outside of deadband, sets variable to stick value
        else {
            // Negative added to fix y-stick values
            // Up on stick is now positive
            rightStickYValue = -m_myController.getY(Hand.kLeft);
        }
        // Returns the stick value
        return rightStickYValue;
    }

    /**
     * Gets the value of the left trigger
     * @return The value of the left trigger axis
     */
    public double getLeftTrigger() {
        double leftTriggerValue;
        // Checks if the trigger value is less than deadband
        if (m_myController.getTriggerAxis(Hand.kLeft) < RobotMap.CONTROLLER_TRIGGER_DEADBAND) {
            // If so, sets the trigger value to 0
            leftTriggerValue = 0.00;
        }
        // If the trigger is outside of deadband, sets the variable to trigger value
        else {
            leftTriggerValue = m_myController.getTriggerAxis(Hand.kLeft);
        }
        // Returns trigger value
        return leftTriggerValue;
    }

    /**
     * Gets the value of the right trigger
     * @return The value of the right trigger axis
     */
    public double getRightTrigger() {
        double rightTriggerValue;
        // Checks if the trigger value is less than deadband
        if (m_myController.getTriggerAxis(Hand.kRight) < RobotMap.CONTROLLER_TRIGGER_DEADBAND) {
            // If so, sets the trigger value to 0
            rightTriggerValue = 0.00;
        }
        // If the trigger is outside of deadband, sets the variable to trigger axis
        else {
            rightTriggerValue = m_myController.getTriggerAxis(Hand.kRight);
        }
        // Returns the trigger value
        return rightTriggerValue;
    }
}