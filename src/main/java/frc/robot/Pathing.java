package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class is what will drive the robot in automated pathing calls
 * @author Josh Overbeek
 * @version Week 0 (Comp Season)
 */
public class Pathing {
    // Doubles for storing the return from the arduino
    private Double m_angleToCenter = Double.NaN;
    private Double m_compareAngleToCenter = Double.NaN;

    // Doubles for storing calcs using the arduino
    private double m_startingDegrees = Double.NaN;
    private double m_absoluteDegToTarget = Double.NaN;
    
    // Flags for running through the code
    private boolean m_lowTargetFound = false;
    private boolean m_rotLowTargetFinished = false;
    private boolean m_lowDriveFinished = false;
    private boolean foundFlag = false;
    private boolean lastCorrectionFlag = false;

	// Declare our duino communication port
	DuinoToRioComms m_duinoToRio;
    private DuinoCommStorage m_pkt;

    // Declare NavX
    NavX m_gyro;
    
    // Declares a drivetrain to use in the auto movement
    Drivetrain m_drivetrain;

    // Counter so we sample data from the duino rather than constantly grab data
    private int m_angleToCenterCompareCounter = 0;

    Controller m_pilotControl;

    private boolean m_onTargetTest = false;
    
    /**
     * Constructor for our pathing sequence, passing in the drivetrain we want to use
     * @param drivetrain The drivetrain for using with the pathing sequence
     */
    public Pathing(Drivetrain drivetrain, NavX ahrs, Controller controller) {
        // Instantiates the duino comms system to get data from the pixys
        m_duinoToRio = new DuinoToRioComms();

        // Instantiates the controller for checking input
        m_pilotControl = controller;

        // Instantiates the drivetrain with the drivetrain passed in
        m_drivetrain = drivetrain;

        // Instantiates the navx with the passed in ahrs
        m_gyro = ahrs;

        // Configs the talon PIDs
        m_drivetrain.talonDriveConfig();
    }

    /**
     * Pathing method based on PixyCam vision for driving to a target
     * 
     * @param lastUltraDist The stopping distance in inches from the target
     * @return Whether or not the method has finished (Target Reached)
     */
    public boolean driveToTarget(int lastUltraDist) {
        
        //  Read angle to center with sampling rate
        if (m_rotLowTargetFinished && (m_angleToCenterCompareCounter >= 25)) {
            m_compareAngleToCenter = m_duinoToRio.getAngleToCenter();
            m_angleToCenterCompareCounter = 0;
        }
        else {
            m_angleToCenterCompareCounter++;
        }

        if ((Math.abs(m_compareAngleToCenter) < RobotMap.STRAIGHT_ANGLE_THRESHOLD) && !m_onTargetTest) {
            System.out.println("On target");
            m_onTargetTest = true;
        }
        else if ((Math.abs(m_compareAngleToCenter) > RobotMap.STRAIGHT_ANGLE_THRESHOLD) && m_onTargetTest) {
            System.out.println("Off target");
            m_onTargetTest = false;
        }

        if((Math.abs(m_compareAngleToCenter) > RobotMap.STRAIGHT_ANGLE_THRESHOLD) && !m_compareAngleToCenter.isNaN() && m_rotLowTargetFinished) {
            if (!lastCorrectionFlag) {
                m_rotLowTargetFinished = false;
                foundFlag = false;
                m_drivetrain.m_firstCallTest = true;
                m_compareAngleToCenter = Double.NaN;
            }
        }

        if (!m_lowTargetFound) {
            m_lowTargetFound = checkForLowTarget();
            return false;
        }
        // Runs the rotLowTarget method after all previous are finished and only if we see a target
        else if (!m_rotLowTargetFinished) {
            m_rotLowTargetFinished = rotLowTarget();
            return false;
        }
        // Runs the driveLowTarget method after all previous are finished
        else if (!m_lowDriveFinished) {
            m_lowDriveFinished = driveLowTarget(lastUltraDist);
            if ( (m_drivetrain.ultraRight.getRangeInches() < (lastUltraDist + 10)) && !lastCorrectionFlag) {
                m_rotLowTargetFinished = false;
                foundFlag = false;
                m_drivetrain.m_firstCallTest = true;
                m_compareAngleToCenter = Double.NaN;
                lastCorrectionFlag = true;
                System.out.println("Last Correction");
            }
            return false;
        }
        // Returns true after all are true
        else {
            m_drivetrain.talonArcadeDrive(0, 0, false);
            return true;
        }
    }

    /**
     * Helper method that checks if the robot can see the low target
     * @return Returns whether the method is finished (True if it is)
     */  
    private boolean checkForLowTarget() {
        if(m_duinoToRio.getLowPosition() == -1) {
            // Returns false if the value returned is the known "No Target" value
            return false;
        }
        else if (m_duinoToRio.getLowPosition() >= 1 && m_duinoToRio.getLowPosition() <= 3) {
            // Returns false if the value returned is expected and seen
            return true;
        }
        else {
            // Returns false if the value is unexpected
            return false;
        }
    }

    /**
     * Helper method that rotates to face the low target
     * @return Returns whether the method is finished (True if it is)
     */
    private boolean rotLowTarget() {
        // System.out.println("Current Heading \t" + m_gyro.getYaw() + "\t Target Heading \t" + m_drivetrain.m_rotController.getSetpoint());
        if (!foundFlag) {
            // Assigns data from duino to a variable
            m_angleToCenter = m_duinoToRio.getAngleToCenter();

            // If the target is a valid number, assigns necesary target variables
            if(!m_angleToCenter.isNaN()) {
                m_startingDegrees = m_gyro.getYaw();
                m_absoluteDegToTarget = m_startingDegrees - (m_angleToCenter);
                foundFlag = true;
            }
            else {
                System.out.println("Looking for target");
            }
            m_onTargetTest = false;
        }

        // Rotates until the method says that its done
        if (foundFlag) {    
            if (m_drivetrain.rotateToAngle(m_absoluteDegToTarget)) {
                m_angleToCenter = Double.NaN;
                m_compareAngleToCenter = Double.NaN;
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }

    /**
     * Method that drives to the low target
     * @return Returns whether the method is finished (True if it is)
     */
    private boolean driveLowTarget(int distance) {
        // Drives forward until within certain distance of the wall
        m_drivetrain.driveToPositionAngle(100, m_absoluteDegToTarget, .2);
        if (m_drivetrain.getRightUltra().getRangeInches() < distance) {
            m_drivetrain.m_firstCallTest = true;
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Resets all sequence flags
     */
    public void resetFlags() {
        m_lowTargetFound = false;
        m_rotLowTargetFinished = false;
        m_lowDriveFinished = false;
        lastCorrectionFlag = false;

        m_drivetrain.m_firstCallTest = true;
        m_drivetrain.m_firstCall = true;

        foundFlag = false;
        
        m_angleToCenter = Double.NaN;
        m_compareAngleToCenter = Double.NaN;

        m_angleToCenterCompareCounter = 0;
    }
}