package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CustomDashboard{
    
    
    
    // Put any numbers / booleans we need to change in here
    public CustomDashboard(){
    
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
        //SmartDashboard.putBoolean("Ball Intake Running" , dataStream.m_ballIntakeRunning);
        
        


    }

}