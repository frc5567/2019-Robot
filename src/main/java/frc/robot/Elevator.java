/**
 * This class defines the mechanism that moves up and down for hatch covers and cargo.
 */

package frc.robot;

import frc.robot.RobotMap;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.MotorSafety;

public class Elevator {

   public enum Height{
      cargoL1 (27.5),        // Height from carpet to port in inches
      cargoL2 (28.0),        // Height from port L1 to L2 in inches
      cargoL3 (28.0),        // Height from port L2 to L3 in inches
      hatchL1 (19.0),        // Height from carpet to hatch in inches
      hatchL2 (28.0),        // Height from hatch L1 to L2 in inches
      hatchL3 (28.0);        // Height from hatch L2 to L3 in inches
      
      Height(double deltaInches){

      }

   }

   public enum PercentMaxRobotSpeed{
      cargoL1 (0.80),         // Percent of max speed we use when elevator is at cargo L1
      cargoL2 (0.60),         // Percent of max speed we use when elevator is at cargo L2
      cargoL3 (0.40),         // Percent of max speed we use when elevator is at cargo L3
      hatchL1 (0.90),         // Percent of max speed we use when elevator is at hatch L1
      hatchL2 (0.65),         // Percent of max speed we use when elevator is at hatch L2
      hatchL3 (0.45);         // Percent of max speed we use when elevator is at hatch L3

      PercentMaxRobotSpeed(double percent){
         
      }
   }

    // Defining the limit switches at the top and bottom of the elevator.
    DigitalInput m_limitTop;
    DigitalInput m_limitBottom;

    // Declaring the encoder for the elevator height.
    Encoder encoder;

    // Declaring the speed controller for the elevator.
    SpeedController raiseElevatorMotor;

    // This constructor is initializing in creating a new instance of an elevator with limit port switch definitions.
   
    Elevator(double distancePerPulse){

        // Creating a new instance of DigitalInput with the assigned port number.
        m_limitTop = RobotMap.m_elevatorLimitTop;
        m_limitBottom = RobotMap.m_elevatorLimitBottom;
        
        // Instantiating encoder for the elevator height
        encoder = RobotMap.m_elevatorEncoder;

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