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
    private boolean m_lowTargetFound = false;
    private boolean m_rotLowTargetFinished = false;
    private boolean m_lowDriveFinished = false;
    private boolean breakFlag = true;
    private boolean lowAutoBreak = true;
    private boolean foundFlag = false;

    private int cycleCounter = 5;

	// Declare our duino communication port
	DuinoToRioComms m_duinoToRio;
    private DuinoCommStorage m_pkt;

    // Declare NavX
    NavX m_gyro;
    
    // Declares a drivetrain to use in the auto movement
    Drivetrain m_drivetrain;

    // Counter so that we only get angle while turning every 15th cycle.
    // Starts at 15 so we collect data the first time through
    private int m_lowDataCollectCounter = 15;

    private int m_rotateExitCounter;
    Controller m_pilotControl;
    
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

        m_rotateExitCounter = 0;
    }

    public boolean secondHalfPath(int lastUltraDist) {

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
            return false;
        }
        // Returns true after all are true
        else {
            m_drivetrain.talonArcadeDrive(.15, 0, false);
            return true;
        }
    }

    /**
     * Converts inches to drive encoder tics
     * @param inches The inches we want to convert
     * @return Returns the resultant tips
     */
    private double inToTics(double inches) {
         return inches*RobotMap.DRIVE_TICS_PER_INCH;
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
        // Collects data and assigns values every 50th cycle
        if (m_lowDataCollectCounter > 14) {
            // Assigns data from duino to a variable
            m_angleToCenter = m_duinoToRio.getAngleToCenter();

            // If the target is a valid number, assigns necesary target variables
            if(!m_angleToCenter.isNaN()) {
                m_startingDegrees = m_gyro.getYaw();
                m_absoluteDegToTarget = m_startingDegrees - (m_angleToCenter);
                
                // Reset the counter
                m_lowDataCollectCounter = 0;
                foundFlag = true;
                System.out.print("starting deg: \t " + m_startingDegrees);
                System.out.print(" degToTarget: \t" + m_angleToCenter);
                System.out.println("Target angle for gyro: \t" + m_absoluteDegToTarget);
            }
        }
        else {
            m_lowDataCollectCounter++;
        }
        // if (!m_angleToCenter.isNaN()) {
            // Rotates until the method says that its done
        if (foundFlag) {    
            if (m_drivetrain.rotateToAngle(m_absoluteDegToTarget)/*m_drivetrain.driveToPositionAngle(24, m_absoluteDegToTarget, .3)*/) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
        // }
        // else {
        //     m_drivetrain.talonArcadeDrive(0.0, 0, false);
        //     return false;
        // }
    }

    /**
     * Method that drives to the low target
     * @return Returns whether the method is finished (True if it is)
     */
    private boolean driveLowTarget(int distance) {
        // Drives forward until within certain distance of the wall
        if (cycleCounter >= 5) {
            m_angleToCenter = m_duinoToRio.getAngleToCenter();

            // If the target is a valid number, assigns necesary target variables
            if(!m_angleToCenter.isNaN()) {
                m_startingDegrees = m_gyro.getYaw();
                m_absoluteDegToTarget = m_startingDegrees - (m_angleToCenter);
                
                // Reset the counter
                cycleCounter = 0;
                foundFlag = true;
            }
        }
        else {
            cycleCounter++;
        }
        m_drivetrain.driveToPositionAngle(100, m_absoluteDegToTarget, .2);
        if(m_drivetrain.getLeftUltra().getRangeInches() < distance || m_drivetrain.getRightUltra().getRangeInches() < distance) {
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

        foundFlag = false;
        m_angleToCenter = Double.NaN;

        cycleCounter = 5;
        m_lowDataCollectCounter = 15;
        m_rotateExitCounter = 0;
    }
}