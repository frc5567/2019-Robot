package frc.robot;

import edu.wpi.first.wpilibj.SPI;

/**
 * This class is what will drive the robot 
 * @author Josh Overbeek
 * @version Week 6 Pre-comp
 */
public class Pathing {
    // Doubles for storing the return from the arduino
	private Double m_degToTarget = Double.NaN;
	private Double m_distToTarget = Double.NaN;
	private Double m_angleToCenter = Double.NaN;
    private Double m_lowPosition = Double.NaN;

    // Doubles for storing calcs using the arduino
    private double m_startingDegrees = Double.NaN;
    private double m_absoluteDegToTarget = Double.NaN;
    private double m_leftInitTics;
    private double m_rightInitTics;
    private double m_leftTargetTics;
    private double m_rightTargetTics;
    private double m_ticsToTarget;
    
    // Flags for running through the code
    private boolean m_foundTarget = false;
    private boolean m_foundLowTarget = false;
    private boolean m_rotEndLineFinished = false;
    private boolean m_foundDistTarget = false;
    private boolean m_driveEndLineFinished = false;
    private boolean m_lowTargetFound = false;
    private boolean m_rotLowTargetFinished = false;
    private boolean m_lowDriveFinished = false;

	// Declare our duino communication port
	private DuinoToRioComms m_duinoToRio;
    private DuinoCommStorage m_pkt;
    
    // Constants for calculating drive distance
    public static final double DRIVE_TICS_PER_INCH = 4096 / (6*RobotMap.PI);
    private final double AUTO_SPEED = 0.3;

    // Declare NavX
    NavX m_ahrs;
    
    // Declares a drivetrain to use in the auto movement
    Drivetrain m_drivetrain;
    
    /**
     * Constructor for our pathing sequence, passing in the drivetrain we want to use
     * @param drivetrain The drivetrain for using with the pathing sequence
     */
    public Pathing(Drivetrain drivetrain) {
        // Instantiates the duino comms system to get data from the pixys
        m_duinoToRio = new DuinoToRioComms();

        // Instantiates the NavX for gyro data
        try {
			m_ahrs = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
        }

        // Instantiates the drivetrain with the drivetrain passed in
        m_drivetrain = drivetrain;

        // Configs the talon PIDs
        m_drivetrain.talonDriveConfig();
    }

    /**
     * Super method for calling all of the helper methods in sequence that should path to target
     * @return Returns whether the method is finished (True if it is)
     */
    public boolean pathToTarget() {
        // Runs the rotEndOfLine method
        if(!m_rotEndLineFinished) {
            m_rotEndLineFinished = rotEndOfLine();
            return false;
        }
        // Runs the driveToLineEnd method after the rotEndLine is finished
        else if (!m_driveEndLineFinished) {
            m_driveEndLineFinished = driveToLineEnd();
            return false;
        }
        // Runs the checkForLowTarget method after all previous are finished
        else if (!m_lowTargetFound) {
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
            m_lowDriveFinished = driveLowTarget();
            return false;
        }
        // Returns true after all are true
        else {
            return true;
        }
    }

    /**
     * Converts inches to drive encoder tics
     * @param inches The inches we want to convert
     * @return Returns the resultant tips
     */
    private double inToTics(double inches) {
         return inches*DRIVE_TICS_PER_INCH;
    }

    /**
     * Helper method to rotate to face the end of the line
     * @return Returns whether the method is finished (True if it is)
     */
    private boolean rotEndOfLine() {
        // Gets data off the arduino only if we haven't found data
        if(!m_foundTarget) {
            // Assigns data from the duino to a storage double
            m_degToTarget = m_duinoToRio.getDegToTarget();

            // Assigns the target for rotation if we have a valid number
            if(!m_degToTarget.isNaN()) {
                m_startingDegrees = m_ahrs.getAngle();
                m_absoluteDegToTarget = m_startingDegrees - m_degToTarget;
                m_foundTarget = true;
            }
            return false;
        }
        else {
            // Returns true if the drive is finished
            if (m_drivetrain.rotateToAngle(m_absoluteDegToTarget)) {
                return true;
            }
            else {
                return false;
            }
        }
    }

    /**
     * Helper method that drives to the end of the line
     * @return Returns whether the method is finished (True if it is)
     */
    private boolean driveToLineEnd() {
        // Gets data off of the arduino only if there is none already
        if (!m_foundDistTarget) {
            // Assigns data from the duino to a storage double
            m_distToTarget = m_duinoToRio.getDistToTarget();

            // If the return value is valid, run needed calculation
            if (!m_distToTarget.isNaN()) {
                m_foundDistTarget = true;
                m_ticsToTarget = inToTics(m_distToTarget);
                m_leftInitTics = m_drivetrain.getLeftDriveEncoderPosition();
                m_rightInitTics = m_drivetrain.getRightDriveEncoderPosition();
                m_leftTargetTics = m_leftInitTics + m_ticsToTarget;
                m_rightTargetTics = m_rightInitTics + m_ticsToTarget;
            }
            return false;
        }
        else {
            // Drives straight if we have not reached our target
            if (m_leftTargetTics < m_drivetrain.getLeftDriveEncoderPosition() && m_rightTargetTics < m_drivetrain.getLeftDriveEncoderPosition()) {
                m_drivetrain.talonArcadeDrive(AUTO_SPEED, 0);
                return false;
            }
            else {
                // Stops the arcade drive otherwise
                m_drivetrain.talonArcadeDrive(0, 0);
                return true;
            }
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
        else if (m_duinoToRio.getLowPosition() >= 1 || m_duinoToRio.getLowPosition() <= 3) {
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
        if(!m_foundLowTarget) {
            //
            m_degToTarget = m_duinoToRio.getAngleToCenter();

            if(!m_degToTarget.isNaN()) {
                m_startingDegrees = m_ahrs.getAngle();
                m_absoluteDegToTarget = m_startingDegrees - m_degToTarget;
                m_foundLowTarget = true;
            }
            return false;
        }
        else {
            if (m_drivetrain.rotateToAngle(m_absoluteDegToTarget)) {
                return true;
            }
            else {
                return false;
            }
        }
    }

    /**
     * Method that drives to the low target
     * @return Returns whether the method is finished (True if it is)
     */
    private boolean driveLowTarget() {
        if(!m_drivetrain.driveToUltra(5)) {
            return false;
        }
        else {
            return true;
        }
    }

    /**
     * Resets all sequence flags
     */
    public void resetFlags() {
        m_foundTarget = false;
        m_foundLowTarget = false;
        m_rotEndLineFinished = false;
        m_foundDistTarget = false;
        m_driveEndLineFinished = false;
        m_lowTargetFound = false;
        m_rotLowTargetFinished = false;
        m_lowDriveFinished = false;
    }

}