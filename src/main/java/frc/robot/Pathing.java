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

    //
    private double m_startingDegrees = Double.NaN;
    private double m_absoluteDegToTarget = Double.NaN;
    private double m_leftInitTics;
    private double m_rightInitTics;
    private double m_leftTargetTics;
    private double m_rightTargetTics;
    private double m_ticsToTarget;
    
    // Flags for running through the code
    private boolean foundTarget = false;
    private boolean foundLowTarget = false;
    private boolean m_rotEndLineFinished = false;
    private boolean m_foundDistTarget = false;
    private boolean m_driveEndLineFinished = false;
    private boolean m_lowTargetFound = false;
    private boolean m_rotLowTargetFinished = false;
    private boolean m_lowDriveFinished = false;

	// Declare our duino communication port
	private DuinoToRioComms m_duinoToRio;
    private DuinoCommStorage m_pkt;
    
    public static final double DRIVE_TICS_PER_INCH = 4096 / (6*RobotMap.PI);
    private final double AUTO_SPEED = 0.3;
    // Declare NavX
    NavX m_ahrs;
    
    Drivetrain m_drivetrain;
	
    public Pathing(Drivetrain drivetrain) {
        m_duinoToRio = new DuinoToRioComms();
        try {
			m_ahrs = new NavX(SPI.Port.kMXP);
		} catch (RuntimeException ex) {
			System.out.println("Error instantiating navX MXP");
        }
        m_drivetrain = drivetrain;
    }

    public boolean pathToTarget() {
        if(!m_rotEndLineFinished) {
            m_rotEndLineFinished = rotEndOfLine();
            return false;
        }
        else if (!m_driveEndLineFinished) {
            m_driveEndLineFinished = driveToLineEnd();
            return false;
        }
        else if (!m_lowTargetFound) {
            m_lowTargetFound = checkForLowTarget();
            return false;
        }
        else if (!m_rotLowTargetFinished) {
            m_rotLowTargetFinished = rotLowTarget();
            return false;
        }
        else if (!m_lowDriveFinished) {
            m_lowDriveFinished = driveLowTarget();
            return false;
        }
        else {
            return true;
        }
    }

    private double inToTics(double inches) {
         return inches*DRIVE_TICS_PER_INCH;
    }

    private boolean rotEndOfLine() {
        //
        if(!foundTarget) {
            //
            m_degToTarget = m_duinoToRio.getDegToTarget();

            if(!m_degToTarget.isNaN()) {
                m_startingDegrees = m_ahrs.getAngle();
                m_absoluteDegToTarget = m_startingDegrees - m_degToTarget;
                foundTarget = true;
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

    private boolean driveToLineEnd() {
        if (!m_foundDistTarget) {
            m_distToTarget = m_duinoToRio.getDistToTarget();
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
            if (m_leftTargetTics < m_drivetrain.getLeftDriveEncoderPosition() && m_rightTargetTics < m_drivetrain.getLeftDriveEncoderPosition()) {
                m_drivetrain.talonArcadeDrive(AUTO_SPEED, 0);
                return false;
            }
            else {
                return true;
            }
        }
    }

    private boolean checkForLowTarget() {
        if(m_duinoToRio.getLowPosition() == -1) {
            return false;
        }
        else if (m_duinoToRio.getLowPosition() >= 1 || m_duinoToRio.getLowPosition() <= 3) {
            return false;
        }
        else {
            return true;
        }
    }

    private boolean rotLowTarget() {
        if(!foundLowTarget) {
            //
            m_degToTarget = m_duinoToRio.getAngleToCenter();

            if(!m_degToTarget.isNaN()) {
                m_startingDegrees = m_ahrs.getAngle();
                m_absoluteDegToTarget = m_startingDegrees - m_degToTarget;
                foundLowTarget = true;
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

    private boolean driveLowTarget() {
        if(!m_drivetrain.driveToUltra(5)) {
            return false;
        }
        else {
            return true;
        }
    }


    // Pull data for deg vector

    // Rotate to new target

    // Switch to approach

    // Check low position, if not negative proceed

    // Perpetually get angle to center and adjust

    // Continue driving forward until ultrasonics say to stop
}