/**
 * This class defines the mechanism that moves up and down for hatch covers and cargo.
 */

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;


public class Elevator {
    // Defining the limit switches at the top and bottom of the elevator.
    DigitalInput m_limitTop;
    DigitalInput m_limitBottom;

    // This constructor is initializing in creating a new instince of an elevator with limit port switch definitions.
    Elevator(int topLimitPort, int bottomLimitPort){
        // Creating a new instince of DigitalInput with the assigned port number.
        m_limitTop = new DigitalInput(topLimitPort);
        m_limitBottom = new DigitalInput(bottomLimitPort);

    }
}