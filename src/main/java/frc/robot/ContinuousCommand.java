package frc.robot;

public class ContinuousCommand {

    // Flags for keeping track of position in sequence
    private boolean m_driveFlag = false;
    private boolean m_rotateFlag = true;
    private boolean m_breakFlag = true;
    
    // Declare NavX
    NavX m_gyro;
    
    // Declares a drivetrain to use in the auto movement
    Drivetrain m_drivetrain;

     /**
     * Constructor for our pathing sequence, passing in the drivetrain we want to use
     * @param drivetrain The drivetrain for using with the pathing sequence
     */
    public ContinuousCommand(Drivetrain drivetrain, NavX ahrs) {
        // Instantiates the drivetrain with the drivetrain passed in
        m_drivetrain = drivetrain;
        // Instantiates the navx with the passed in ahrs
        m_gyro = ahrs;

        // Configs the talon PIDs
        m_drivetrain.talonDriveConfig();
    }

    public void loop(boolean toggle) {
        if (toggle) {
            m_breakFlag = !m_breakFlag;
            m_drivetrain.talonArcadeDrive(0, 0);
            m_driveFlag = false;
            m_rotateFlag = true;
            System.out.println("Toggled");
        }

        if (m_breakFlag) {
            return;
        }

        if (!m_rotateFlag) {
            m_rotateFlag = m_drivetrain.driveToPosition(-8);
            //System.out.println("Rotating");
            if(m_rotateFlag) {
                //System.out.println("Done Rotating");
                m_driveFlag = false;
            }
        }
        else if (!m_driveFlag) {
            m_driveFlag = m_drivetrain.driveToPosition(8);
            //System.out.println("Driving");
            if(m_driveFlag) {
                //System.out.println("Done Driving");
                m_rotateFlag = false;
            }
        }
    }

}