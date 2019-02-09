package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class CustomDashboard{
    
    SendableChooser<String> m_gamePieceSelector = new SendableChooser<>();
    String m_gamePieceSelected;
    Boolean m_hatchSelected;
    Boolean m_cargoSelected;
    
    
    
    // Put any numbers / booleans we need to change in here
    public CustomDashboard(){



    }

    public enum HatchElevatorState {
        kInitial(0),
        kTransition(1),
        kLowPosition(2),
        kMidPosition(3),
        kHighPosition(4),
        kMaxHeight(5);

        private int hatchElevatorState;

        HatchElevatorState(int hatchElevatorState) {
            this.setHatchElevatorState(hatchElevatorState);
        }

        public int getHatchElevatorState(){
            return hatchElevatorState;
        }

        public void setHatchElevatorState(double elevatorHeight){
            
            // PLACEHOLDER VALUES
            double lowThreshhold = 0;
            double midThreshhold = 1;
            double highThreshhold = 2;
            final double MAX_ELEVATOR_HEIGHT = 5.0;

            if(elevatorHeight < lowThreshhold + 2 && elevatorHeight > lowThreshhold - 2){
                this.hatchElevatorState = 2;
            }
            else if(elevatorHeight < midThreshhold + 2 && elevatorHeight > midThreshhold - 2){
                this.hatchElevatorState = 3;
            }
            else if(elevatorHeight < highThreshhold + 2 && elevatorHeight > highThreshhold - 2){
                this.hatchElevatorState = 4;
            }
            else if(elevatorHeight == MAX_ELEVATOR_HEIGHT){
                this.hatchElevatorState = 5;
            }
            else{
                this.hatchElevatorState = 1;
            }
            

        }


    }

    /**
     * Loop this method constantly to update the values on Shuffleboard
     * 
     * @param dataStream Use the DashboardData object we create.
     */

    public void DashboardUpdate(DashboardData dataStream){

        // Values Pertaining to the Elevator
        SmartDashboard.putNumber("Elevator Height" , dataStream.m_rawElevatorHeight);
        
        // Values Associated with the Wheel Encoders
        SmartDashboard.putNumber("Right Wheels Velocity" , dataStream.m_velocityRight);
        SmartDashboard.putNumber("Left Wheels Velocity" , dataStream.m_velocityLeft);

        // Values Associated with Auton Functions
        SmartDashboard.putBoolean("Driver Assist Enabled" , dataStream.m_driverAssist);
        SmartDashboard.putBoolean("Driver Assist Activated" , dataStream.m_autonActive);
        SmartDashboard.putBoolean("Auton Reading Target" , dataStream.m_readingTarget);
        
        // Values Associated with the Grabber
        SmartDashboard.putBoolean("Grabber Open" , dataStream.m_grabberOpen);
        SmartDashboard.putNumber("Grabber Angle", dataStream.m_rawGrabberAngle);

        // Misc Values
        SmartDashboard.putString("Proximity to Nearest Wall" , dataStream.m_proximity);
        SmartDashboard.putNumber("Climber Position" , dataStream.m_climberEncoder);
        SmartDashboard.putBoolean("Ball Intake Running" , dataStream.m_ballIntakeRunning);
        
        


    }

}