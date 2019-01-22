/**
 * This class defines the mechanism that moves up and down for hatch covers and cargo.
 */

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;

public class Elevator {
    // Defining the limit switches at the top and bottom of the elevator.
    DigitalInput m_limitTop;
    DigitalInput m_limitBottom;

    // Declaring the encoder for the elevator height.
    Encoder elevatorEncoder;

    // Declaring the speed controller for the elevator.
    SpeedController raiseElevatorMotor;

    // This constructor is initializing in creating a new instance of an elevator with limit port switch definitions.
    Elevator(int topLimitPort, int bottomLimitPort){

        // Creating a new instance of DigitalInput with the assigned port number.
        m_limitTop = new DigitalInput(topLimitPort);
        m_limitBottom = new DigitalInput(bottomLimitPort);
    }    
        // Instantiating encoder for the elevator height
        elevatorEncoder = new Encoder(2, 1, false, EncodingType.k1X);

    
}