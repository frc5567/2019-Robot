package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Controller extends XboxController {

    double leftStickXValue;
    double leftStickYValue;
    double rightStickXValue;
    double rightStickYValue;
    double leftTriggerValue;
    double rightTriggerValue;

    XboxController myController;

    static final double kStickDeadband = 0.05;
    static final double kTriggerDeadband = 0.05;

    Controller(int port) {
        super(port);

        myController = new XboxController(port);
    }

    public double getLeftStickX() {
        if ((myController.getX(Hand.kLeft) < kStickDeadband) && (myController.getX(Hand.kLeft) > -kStickDeadband)) {
            leftStickXValue = 0.00;
        }
        else {
            leftStickXValue = myController.getX(Hand.kLeft);
        }
        return leftStickXValue;
    }

    public double getLeftStickY() {
        if ((myController.getY(Hand.kLeft) < kStickDeadband) && (myController.getY(Hand.kLeft) > -kStickDeadband)) {
            leftStickYValue = 0.00;
        }
        else {
            leftStickYValue = myController.getY(Hand.kLeft);
        }
        return leftStickYValue;
    }

    public double getRighStickX() {
        if ((myController.getX(Hand.kRight) < kStickDeadband) && (myController.getX(Hand.kRight) > -kStickDeadband)) {
            rightStickXValue = 0.00;
        }
        else {
            rightStickXValue = myController.getX(Hand.kRight);
        }
        return rightStickXValue;
    }

    public double getRighStickY() {
        if ((myController.getY(Hand.kRight) < kStickDeadband) && (myController.getY(Hand.kRight) > -kStickDeadband)) {
            rightStickYValue = 0.00;
        }
        else {
            rightStickYValue = myController.getY(Hand.kLeft);
        }
        return rightStickYValue;
    }

    public double getLeftTrigger() {
        if (myController.getTriggerAxis(Hand.kLeft) < kTriggerDeadband) {
            leftTriggerValue = 0.00;
        }
        else {
            leftTriggerValue = myController.getTriggerAxis(Hand.kLeft);
        }
        return leftTriggerValue;
    }

    public double getRightTrigger() {
        if (myController.getTriggerAxis(Hand.kRight) < kTriggerDeadband) {
            rightTriggerValue = 0.00;
        }
        else {
            rightTriggerValue = myController.getTriggerAxis(Hand.kRight);
        }
        return rightTriggerValue;
    }
}