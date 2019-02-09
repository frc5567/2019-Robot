package frc.robot;

public class StateMachine {

    /**
     * TODO: Add state for retireval auto and placement auto Add transition states
     * to smoothen switch between auto and teleop
     */
    enum States {
        DISABLED, // Prematch / postmatch
        TELEOP, // User control
        AUTO, // Pure auto, used for beginning of match
        AUTO_ASSIST, // Auto mode for game piece pickup & scoring
        ERROR; // Error failsafe, might be used if sensors malfunction
    }

    enum AutoStates {
        NONE,

        LEFT_LVL1_CLOSE_ROCKET, LEFT_LVL2_CLOSE_ROCKET, RIGHT_LVL1_CLOSE_ROCKET, RIGHT_LVL2_CLOSE_ROCKET,

        LEFT_LVL1_FAR_ROCKET, LEFT_LVL2_FAR_ROCKET, RIGHT_LVL1_FAR_ROCKET, RIGHT_LVL2_FAR_ROCKET
    }

    // Variable to store current robot state
    States currentState;
    // Stores auto states for beginning of match
    AutoStates currentAutoState;

    Controller m_pilotController;

    NavX m_ahrs;

    Drivetrain m_drivetrain;

    /**
     * State Machines handle the overall state of the robot This state machine is
     * deisgned to run in Teleop and Auto mode for lining up for scoring
     */
    StateMachine(Controller pilotController, NavX ahrs, Drivetrain drivetrain) {
        currentState = States.DISABLED;
        currentAutoState = AutoStates.NONE;
        m_pilotController = pilotController;
        m_ahrs = ahrs;
        m_drivetrain = drivetrain;
    }

    /**
     * Returns the current state of the State Machine, useful if current state is
     * going to be published on dashboard
     * 
     * @return The current state of the state machine
     */
    public States getCurrentState() {
        return currentState;
    }

    /**
     * Updates the state machine to the state desired
     * 
     * @param nextState The state the state machine will switch to next TODO: Add
     *                  logic to only switch to auto when conditions are correct
     *                  (user set and sensors have target)
     */
    public void updateState(States nextState) {
        currentState = nextState;
    }

    public void updateAutoState(AutoStates nextState) {

    }

    /**
     * TODO: Add methods to get controller inputs for TELEOP mode || either set methods or call Controller methods
     * Add methods to get sensor data to check if
     * targets are found || Might be calling methods from sensors subsystems
     * Add methods to use for AUTO sequencing of subsystems || If actions are unique for each part, will use switch statements
     * Add methods to verify sensor readings,  reports if sensor is malfunctioning || If ERROR state not used, will be removed
     * Add methods to publish data to dashboard (state and errors) || If state machine data is not being published to dashboard, will be removed
     * Add methods to disable / reenable sensors when neccessary to reduce amount of polling requests in TELEOP state
     */

    /**
     * The main state machine TODO: Finalize placement of state machine Coordinate
     * on subsystems for interacting with the subsystems in auto and transitioning
     * states
     */
    public void FSM() {
        switch (currentState) {
        case DISABLED:
            /**
             * TODO: Figure out if disabled state is needed (Disable sensors / motors or
             * verify everything is functioning before start)
             */
            break;
        case TELEOP:
            /**
             * TODO: Add method calls for teleop as drivers will control robot as normal Add
             * method call for sensors to check if target is found and drivers hit button to
             * initiate auto Disable ultrasonics during TELEOP until needed in AUTO to
             * prevent unneccessary use of ultrasonics in TELEOP
             */

            // Test drivetrain included, uses Left stick Y for speed, Right stick X for
            // turning, quick turn is auto-enabled at low speed
            m_drivetrain.curvatureDrive(m_pilotController.getLeftStickY(), m_pilotController.getRightStickX());

            if (m_pilotController.getAButtonReleased()) {
                m_ahrs.zeroYaw();
            }
            if (m_pilotController.getBButtonReleased()) {
                m_ahrs.flipOffset();
            }

            System.out.println(m_ahrs.getOffsetYaw() + "\t\t" + m_ahrs.getOffsetStatus());
            break;
        case AUTO:
            /**
             * TODO: Split into two states for retirieval and placement of game pieces Add
             * method call for each step of AUTO state to controll subsystems
             */
            switch (currentAutoState) {
            case NONE:
                /**
                 * Used as 'default' until the team establishes another case for default
                 */
                break;

            case LEFT_LVL1_CLOSE_ROCKET:
                // Robot is on left on level 1 platform, going for close rocket hatch
                break;
            case LEFT_LVL2_CLOSE_ROCKET:
                // Robot is on left on level 2 platform, going for close rocket hatch
                break;
            case RIGHT_LVL1_CLOSE_ROCKET:
                // Robot is on right on level 1 platform, going for close rocket hatch
                break;
            case RIGHT_LVL2_CLOSE_ROCKET:
                // Robot is on right on level 2 platform, going for close rocket hatch
                break;

            case LEFT_LVL1_FAR_ROCKET:
                // Robot is on left on level 1 platform, going for far rocket hatch
                break;
            case LEFT_LVL2_FAR_ROCKET:
                // Robot is on left on level 2 platform, going for far rocket hatch
                break;
            case RIGHT_LVL1_FAR_ROCKET:
                // RObot is on right on level 1 platform, going for far rocket hatch
                break;
            case RIGHT_LVL2_FAR_ROCKET:
                // Robot is on right on level 2 platform, going for far rocket hatch
                break;
            default:
                /**
                 * TODO: Default used as failsafe, might call updateAutoState to check if data
                 * is being called, or it might enter error state until resolved
                 */
            }
            break;
        case AUTO_ASSIST:

            break;
        case ERROR:
            /**
             * TODO: If used for sensor checking, state will only be used if sensors are
             * unresponsive (disconnected) If sensor is required for AUTO, then ERROR state
             * will lock out AUTO and possibly report error to dashboard If sensor does not
             * effect AUTO, then error is possibly reported to dashboard, and AUTO is not
             * interrupted
             */
            break;
        default:
            /**
             * TODO: Default might be used as failsafe if invalid state somehow gets
             * through, default will probably revert state to either TELEOP or ERROR state
             * for invalid state
             */
        }
    }
}