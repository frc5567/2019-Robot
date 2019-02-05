package frc.robot;

public class StateMachine {

    /**
     * TODO:
     * Add state for retireval auto and placement auto
     * Add transition states to smoothen switch between auto and teleop
     */
    enum States {
        DISABLED, // Prematch / postmatch
        TELEOP, // User control
        AUTO, // Auto mode, used for game piece pickup & scoring
        ERROR; // Error failsafe, might be used if sensors malfunction
    }

    // Variable to store current robot state
    States currentState;

    /**
     * State Machines handle the overall state of the robot
     * This state machine is deisgned to run in Teleop and Auto mode for lining up for scoring
     */
    StateMachine() {
        currentState = States.DISABLED;
    }

    /**
     * Returns the current state of the State Machine, useful if current state is going
     * to be published on dashboard
     * @return The current state of the state machine
     */
    public States getCurrentState() {
        return currentState;
    }

    /**
     * Updates the state machine to the state desired
     * @param nextState The state the state machine will switch to next
     * TODO:
     * Add logic to only switch to auto when conditions are correct (user set and sensors have target)
     */
    public void updateState(States nextState) {
        currentState = nextState;
    }

    /**
     * TODO:
     * Add methods to get controller inputs for TELEOP mode     || either set methods or call Controller methods
     * Add methods to get sensor data to check if targets are found     || Might be calling methods from sensors subsystems
     * Add methods to use for AUTO sequencing of subsystems     || If actions are unique for eeach part, will use switch statements
     * Add methods to verify sensor readings, reports if sensor is malfunctioning   || If ERROR state not used, will be removed
     * Add methods to publish data to dashboard (state and errors)      || If state machine data is not being published to dashboard, will be removed
     * Add methods to disable / reenable sensors when neccessary to reduce amount of polling requests in TELEOP state
     */

    /**
     * The main state machine
     * TODO:
     * Finalize placement of state machine
     * Coordinate on subsystems for interacting with the subsystems in auto and transitioning states
     */
    public void FSM() {
        switch (currentState) {
            case DISABLED:
            /**
             * TODO:
             * Figure out if disabled state is needed (Disable sensors / motors or verify everything is functioning before start)
             */
            break;
            case TELEOP:
            /**
             * TODO:
             * Add method calls for teleop as drivers will control robot as normal
             * Add method call for sensors to check if target is found and drivers hit button to initiate auto
             * Disable ultrasonics during TELEOP until needed in AUTO to prevent unneccessary use of ultrasonics in TELEOP
             */
            break;
            case AUTO:
            /**
             * TODO:
             * Split into two states for retirieval and placement of game pieces
             * Add method call for each step of AUTO state to controll subsystems
             */
            break;
            case ERROR:
            /**
             * TODO:
             * If used for sensor checking, state will only be used if sensors are unresponsive (disconnected)
             * If sensor is required for AUTO, then ERROR state will lock out AUTO and possibly report error to dashboard
             * If sensor does not effect AUTO, then error is possibly reported to dashboard, and AUTO is not interrupted
             */
            break;
            default:
            /**
             * TODO:
             * Default might be used as failsafe if invalid state somehow gets through, 
             * default will probably revert state to either TELEOP or ERROR state for invalid state
             */
        }
    }
}