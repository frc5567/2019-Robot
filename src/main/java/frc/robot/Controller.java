package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controller extends XboxController {

    // Declares values used to return value of controller sticks and triggers
    double m_leftStickXValue;
    double m_leftStickYValue;
    double m_rightStickXValue;
    double m_rightStickYValue;
    double m_leftTriggerValue;
    double m_rightTriggerValue;

    // Declares xbox controller used by Controller object
    XboxController m_myController;

    // Sets constants used for deadbands, one for sticks and the other for triggers
    static final double kStickDeadband = 0.05;
    static final double kTriggerDeadband = 0.05;

    /**
     * Handles for input from driver's Xbox Controllers, and abjusts analog input using deadbands
     * @param port The Driver Station USB the controller is plugged into
     */
    Controller(int port) {
        super(port);

        // Initializes controller to port passed in contructor
        m_myController = new XboxController(port);
    }

    /**
     * Gets the value of the left stick X axis
     * @return The X value of the left stick
     */
    public double getLeftStickX() {
        // Checks if stick is within deadband
        if ((m_myController.getX(Hand.kLeft) < kStickDeadband) && (m_myController.getX(Hand.kLeft) > -kStickDeadband)) {
            // If so, stick value is set to 0
            m_leftStickXValue = 0.00;
        }
        // If the stick is outside deadband, sets variable to stick value
        else {
            m_leftStickXValue = m_myController.getX(Hand.kLeft);
        }
        // Returns the stick value
        return m_leftStickXValue;
    }

    /**
     * Gets the value of the left stick Y axis
     * @return The Y value of the left axis
     */
    public double getLeftStickY() {
        // Checks if stick value is within deadband
        if ((m_myController.getY(Hand.kLeft) < kStickDeadband) && (m_myController.getY(Hand.kLeft) > -kStickDeadband)) {
            // If so, stick value is set to 0
            m_leftStickYValue = 0.00;
        }
        // If the stick is outside deadband, sets variable to stick value
        else {
            m_leftStickYValue = m_myController.getY(Hand.kLeft);
        }
        // Returns the stick value
        return m_leftStickYValue;
    }

    /**
     * Gets the value of the right stick X axis
     * @return The X value of the right axis
     */
    public double getRighStickX() {
        // Checks if the stick value is within deadband
        if ((m_myController.getX(Hand.kRight) < kStickDeadband) && (m_myController.getX(Hand.kRight) > -kStickDeadband)) {
            // If so, set stick value to 0
            m_rightStickXValue = 0.00;
        }
        // If the stick is outside deadband, sets variable to stick value
        else {
            m_rightStickXValue = m_myController.getX(Hand.kRight);
        }
        // Returns the stick value
        return m_rightStickXValue;
    }

    /**
     * Gets the value of the right stick Y axis
     * @return The Y value of the right axis
     */
    public double getRighStickY() {
        // Checks if the stick value is within deadband
        if ((m_myController.getY(Hand.kRight) < kStickDeadband) && (m_myController.getY(Hand.kRight) > -kStickDeadband)) {
            // If so, set stick value to 0
            m_rightStickYValue = 0.00;
        }
        // If the stick is outside of deadband, sets variable to stick value
        else {
            m_rightStickYValue = m_myController.getY(Hand.kLeft);
        }
        // Returns the stick value
        return m_rightStickYValue;
    }

    /**
     * Gets the value of the left trigger
     * @return The value of the left trigger axis
     */
    public double getLeftTrigger() {
        // Checks if the trigger value is less than deadband
        if (m_myController.getTriggerAxis(Hand.kLeft) < kTriggerDeadband) {
            // If so, sets the trigger value to 0
            m_leftTriggerValue = 0.00;
        }
        // If the trigger is outside of deadband, sets the variable to trigger value
        else {
            m_leftTriggerValue = m_myController.getTriggerAxis(Hand.kLeft);
        }
        // Returns trigger value
        return m_leftTriggerValue;
    }

    /**
     * Gets the value of the right trigger
     * @return The value of the right trigger axis
     */
    public double getRightTrigger() {
        // Checks if the trigger value is less than deadband
        if (m_myController.getTriggerAxis(Hand.kRight) < kTriggerDeadband) {
            // If so, sets the trigger value to 0
            m_rightTriggerValue = 0.00;
        }
        // If the trigger is outside of deadband, sets the variable to trigger axis
        else {
            m_rightTriggerValue = m_myController.getTriggerAxis(Hand.kRight);
        }
        // Returns the trigger value
        return m_rightTriggerValue;
    }
}