package frc.robot;

public class DashboardData{

    double m_rawElevatorHeight;
    double m_velocityRight;
    double m_velocityLeft;
    String m_proximity;
    String proximityFormat;
    boolean m_driverAssist;
    boolean m_grabberOpen;
    double m_rawGrabberAngle;
    double m_climberEncoder;
    boolean m_autonActive;
    boolean m_readingTarget;
    boolean m_ballIntakeRunning;
    String m_elevatorState;


    /**
     * Constructor used as a data bridge between the sensors and the CustomDashboard class.
     * 
     * @param elevatorEncoder The value read by calcPosition() of our elevator object.
     * @param encoderRateRight Value read by .getRate() of our right encoder.
     * @param encoderRateLeft Value read by .getRate() of our left encoder.
     * @param driverAssistOn Boolean that tests whether or not Driver Assist is being used.
     * @param proximitySensorInches The reading of our proximity sensor, in inches.
     * @param grabberOpen True if the grabber is in the open position, otherwise false.
     * @param grabberAngle Angle of the grabber arm rwad by our encoder.
     * @param climberPosition The position of our climber based on it's encoder.
     * @param autonActivated True if auton is engaged, if disengaged it will read false.
     * @param autonReadingTarget True if the Pixy detects a line below the robot.
     * @param ballIntakeRunning True if our ball intake system is currently running.
     * @param elevatorState Value read from getState() method of our elevator object.
     */
    
    public DashboardData(double elevatorEncoder , double encoderRateRight , double encoderRateLeft ,
    boolean driverAssistOn , double proximitySensorInches , boolean grabberOpen , double grabberAngle ,
    double climberPosition , boolean autonActivated , boolean autonReadingTarget , boolean ballIntakeRunning , 
    String elevatorState){
        m_rawElevatorHeight = elevatorEncoder;
        
        m_velocityRight = encoderRateRight;
        m_velocityLeft = encoderRateLeft;
        
        m_driverAssist = driverAssistOn;
        
        /* Statement used to format the output of our proximity sensor, rounding it to 4 decimal
         * points and adding the unit.
         */ 
        if(proximitySensorInches < 12.0){
            proximityFormat = Double.toString(proximitySensorInches);
            proximityFormat = proximityFormat.substring(0 , proximityFormat.indexOf(".") + 4);
            m_proximity = (proximityFormat+ " inches");
        }
        else if(proximitySensorInches >= 12.0){
            proximitySensorInches = proximitySensorInches / 12;
            proximityFormat = Double.toString(proximitySensorInches);
            proximityFormat = proximityFormat.substring(0 , proximityFormat.indexOf(".") + 4);
            m_proximity = (proximityFormat+ " feet");
        }

        m_grabberOpen = grabberOpen;

        m_rawGrabberAngle = grabberAngle;

        m_climberEncoder = climberPosition;

        m_autonActive = autonActivated;

        m_readingTarget = autonReadingTarget;

        m_ballIntakeRunning = ballIntakeRunning;

        m_elevatorState = elevatorState;

        
    }

}