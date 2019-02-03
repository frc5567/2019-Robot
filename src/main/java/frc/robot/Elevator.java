/**
 * This class defines the mechanism that moves up and down for hatch covers and cargo.
 */

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.MotorSafety;

public class Elevator {

   public enum State{
      LEVEL_ZERO  (0.0 , 1.0 , 1.0),
      CARGO_L1 (27.5 , 0.80 , 0.50),
      CARGO_L2 (28.0 , 0.60 , 0.30),
      CARGO_L3 (28.0 , 0.40 , 0.20),
      HATCH_L1 (19.0 , 0.90 , 0.50),
      HATCH_L2 (28.0 , 0.65 , 0.30),
      HATCH_L3 (28.0 , 0.45 , 0.20);

      private double deltaHeightInches;
      private double maxSpeedPercent;
      private double maxAngleRate;

      /**
       * 
       * 
       * @param deltaInches Change in height from position zero in inches.
       * @param maxSpeedModifier Percent of our max speed we use when the elevator is in this state.
       * @param maxAngleRate How fast our angle motor can move when the elevator is in this state.
       */
      State(double deltaInches , double maxSpeedModifier , double maxAngleRate){
         this.deltaHeightInches = deltaInches;
         this.maxSpeedPercent = maxSpeedModifier;
         this.maxAngleRate = maxAngleRate;
      }

      /**
       * @return The change in height from initial to current state.
       */
      public double getDeltaHeight(){
         return this.deltaHeightInches;
      }

      /**
       * @return The max speed modifier based on the elevator's current state.
       */
      public double maxSpeedModifier(){
         return this.maxSpeedPercent;
      }

      /**
       * @return The max speed we turn our angle arm at the elevator's current state.
       */
      public double maxAngleRate(){
         return this.maxAngleRate;
      }
   }

   // Defining the limit switches at the top and bottom of the elevator.
   DigitalInput m_limitTop;
   DigitalInput m_limitBottom;

   // Declaring the encoder for the elevator height.
   Encoder encoder;

   // Declaring the speed controller for the elevator.
   SpeedController raiseElevatorMotor;

   // Declaring the elevator state enum.
   State currentState;

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