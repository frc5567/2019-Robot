/**
 * This class defines the mechanism that moves up and down for hatch covers and cargo.
 */

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.MotorSafety;

public class Elevator {
    // Defining the limit switches at the top and bottom of the elevator.
    DigitalInput m_limitTop;
    DigitalInput m_limitBottom;

    // Declaring the encoder for the elevator height.
    Encoder encoder;

    // Declaring the speed controller for the elevator.
    SpeedController raiseElevatorMotor;

    // This constructor is initializing in creating a new instance of an elevator with limit port switch definitions.
   
    Elevator(int topLimitPort, int bottomLimitPort, double distancePerPulse){

        // Creating a new instance of DigitalInput with the assigned port number.
        m_limitTop = new DigitalInput(topLimitPort);
        m_limitBottom = new DigitalInput(bottomLimitPort);
        
        // Instantiating encoder for the elevator height
        encoder = new Encoder(2, 1, false, Encoder.EncodingType.k1X);

        //  Sets the distance for each encoder pulse
        encoder.setDistancePerPulse(distancePerPulse);
        
        //  Sets elevator height variable equal to the encoder distance
     //   double m_elevatorHeight = encoder.getDistance();

    }
    
    // defines elevator height 
 public void getHeight(double m_elevatorHeight){

    m_elevatorHeight = encoder.getDistance();
    double START_HEIGHT = 4;
   double m_totalElevatorHeight = m_elevatorHeight + START_HEIGHT;
   System.out.println (m_totalElevatorHeight);

 }    




}