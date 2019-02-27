package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

/**
 * This class is what will drive the robot in automated pathing calls
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
    private boolean breakFlag = true;
    private boolean lowAutoBreak = true;

	// Declare our duino communication port
	DuinoToRioComms m_duinoToRio;
    private DuinoCommStorage m_pkt;

    // Declare NavX
    NavX m_gyro;
    
    // Declares a drivetrain to use in the auto movement
    Drivetrain m_drivetrain;

    int m_counter;
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

        m_counter = 0;
    }

    /**
     * Super method for calling all of the helper methods in sequence that should path to target
     * @return Returns whether the method is finished (True if it is)
     */
    public boolean pathToTarget() {
        // Runs the rotEndOfLine method
        if(!m_rotEndLineFinished) {
            System.out.println("rot end line");
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
            System.out.println("low target found");
            m_lowTargetFound = checkForLowTarget();
            return false;
        }
        // Runs the rotLowTarget method after all previous are finished and only if we see a target
        else if (!m_rotLowTargetFinished) {
            System.out.println("rot low target");
            m_rotLowTargetFinished = rotLowTarget();
            return false;
        }
        // Runs the driveLowTarget method after all previous are finished
        else if (!m_lowDriveFinished) {
            System.out.println("low drive");
            m_lowDriveFinished = driveLowTarget();
            return false;
        }
        // Returns true after all are true
        else {
            System.out.println("finished");
            return true;
        }
    }

    public boolean secondHalfPath(boolean toggle) {
        if (toggle) {
            lowAutoBreak = !lowAutoBreak;
        }

        if (lowAutoBreak) {
            if ( m_pilotControl.getLeftTrigger() > 0 || m_pilotControl.getRightTrigger() > 0 || m_pilotControl.getLeftStickX() != 0) {
                return false;
            }
            else {
                m_drivetrain.talonArcadeDrive(0, 0);
                return false;
            }
        }

        if (!m_lowTargetFound) {
            m_lowTargetFound = checkForLowTarget();
            System.out.println("low target: \t" + m_lowTargetFound);
            return false;
        }
        // Runs the rotLowTarget method after all previous are finished and only if we see a target
        else if (!m_rotLowTargetFinished) {
            System.out.println("rot low target");
            m_rotLowTargetFinished = rotLowTarget();
            return false;
        }
        // Runs the driveLowTarget method after all previous are finished
        else if (!m_lowDriveFinished) {
            System.out.println("low drive");
            m_lowDriveFinished = driveLowTarget();
            return false;
        }
        // Returns true after all are true
        else {
            System.out.println("finished");
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
     * Helper method to rotate to face the end of the line
     * @return Returns whether the method is finished (True if it is)
     */
    private boolean rotEndOfLine() {
        // Gets data off the arduino only if we haven't found data
        if (!m_foundTarget) {
            // Assigns data from the duino to a storage double
            m_degToTarget = m_duinoToRio.getDegToTarget();
            System.out.println(m_degToTarget);

            // Assigns the target for rotation if we have a valid number
            if (m_degToTarget == -180) {
                System.out.println("No target found");
            }
            else if (!m_degToTarget.isNaN()) {
                m_startingDegrees = m_gyro.getYaw();
                m_absoluteDegToTarget = m_startingDegrees + m_degToTarget;
                m_foundTarget = true;
            }
            return false;
        }
        else {
            // Returns true if the drive is finished
            System.out.println("Current Angle : \t" + m_gyro.getYaw());
            System.out.println("Target Angle: \t" + m_absoluteDegToTarget);
            System.out.println("PIDOutput: \t" + m_drivetrain.m_rotController.get());
            m_drivetrain.rotateToAngle(m_absoluteDegToTarget);
            System.out.println("Post method PIDOutput: \t" + m_drivetrain.m_rotController.get());
            if ((m_counter > 25) && ((m_drivetrain.m_rotController.get() < RobotMap.FINISHED_PID_THRESHOLD) && (m_drivetrain.m_rotController.get() > -RobotMap.FINISHED_PID_THRESHOLD))) {
                return true;
            }
            else {
                m_counter++;
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
            m_distToTarget = Math.abs(m_duinoToRio.getDistToTarget());
            System.out.println(m_distToTarget);

            // If the return value is valid, run needed calculation
            if (!m_distToTarget.isNaN()) {
                m_foundDistTarget = true;
                m_ticsToTarget = inToTics(m_distToTarget);
                m_leftInitTics = m_drivetrain.getLeftDriveEncoderPosition();
                m_rightInitTics = m_drivetrain.getRightDriveEncoderPosition();
                m_leftTargetTics = m_leftInitTics - m_ticsToTarget;
                m_rightTargetTics = m_rightInitTics - m_ticsToTarget;
            }
            System.out.println("First entry");
            return false;
        }
        else {
            // Drives straight if we have not reached our target
            if (m_leftTargetTics < m_drivetrain.getLeftDriveEncoderPosition() && m_rightTargetTics < m_drivetrain.getLeftDriveEncoderPosition()) {
                m_drivetrain.talonArcadeDrive(RobotMap.AUTO_SPEED, 0);
                return false;
            }
            else {
                // Stops the arcade drive otherwise
                m_drivetrain.talonArcadeDrive(0, 0);
                System.out.println("Done driving");
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
        System.out.println("target angle: \t" + m_absoluteDegToTarget);
        System.out.println("current angle: \t" + m_gyro.getYaw());
        // Assigns the target if there is no previous valid target
        m_angleToCenter = m_duinoToRio.getAngleToCenter();
        System.out.println("Looking for target");
        // If the target is a valid number, assigns necesary target variables
        if(!m_angleToCenter.isNaN()) {
            System.out.println("Found Target");
            m_startingDegrees = m_gyro.getYaw();
            m_absoluteDegToTarget = m_startingDegrees + m_angleToCenter;
        }
        
        // This code is designed to shift drive method into rotate without forward movement if the robot was too far off target
        // TODO: Find if this is necesary through testing and either update or remove this code on necesity
        // if (!m_angleToCenter.isNaN() && Math.abs(m_angleToCenter) > 30){
        //     // Rotates until the method says that its done
        //     if ((m_drivetrain.rotateToAngle(m_absoluteDegToTarget))) {
        //         System.out.println("Done Rotating");
        //         return true;
        //     }
        //     else {
        //         System.out.println("Rotating");
        //         return false;
        //     }
        // }

        if (!m_angleToCenter.isNaN()) {
            // Rotates until the method says that its done
            if (m_drivetrain.rotateDriveAngle(m_absoluteDegToTarget, m_duinoToRio.getAverageArea())) {
                System.out.println("Done Rotating");
                return true;
            }
            else {
                System.out.println("Rotating");
                return false;
            }
        }
        else {
            m_drivetrain.talonArcadeDrive(0, 0);
            return false;
        }
    }

    /**
     * Method that drives to the low target
     * @return Returns whether the method is finished (True if it is)
     */
    private boolean driveLowTarget() {
        // Drives forward until within certain distance of the wall
        if(!m_drivetrain.driveToUltra(18)) {
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