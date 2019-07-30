package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

import frc.robot.Controller;
import frc.robot.GamePad;
import frc.robot.Elevator.State;
import frc.robot.Drivetrain;
import frc.robot.Elevator;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.Climber;
import frc.robot.HatchMech;

/**
 * This class is to house all the commands used by the pilots in teleop mode
 * 
 * @author Matt
 * @version Last Minute. Verrry Last Minute.
 */
public class TeleopCommands {

    // Declares variables for the controller and gamepad the pilot and copilot use.
    Controller m_controller;
    GamePad m_gamepad;

    // Declares variable for the main drivetrain
    Drivetrain m_drivetrain;

    // Declares variables for the elevator subsystem
    Elevator m_elevator;

    // Declares variables for the front and back climber, as well as the PID that controls them when ascendinng
    Climber m_frontClimber;
    DriveClimber m_backClimber;
    ClimberPIDControl m_climberPID;

    // Declares variable for hatchmech subsystem
    HatchMech m_hatchMech;

    // Declares a boolean to see if the back climber is deployed
    // used to prevent the drive motor from running when retracted
    boolean m_driveClimberDeployed = false;

    // Declares a variable for the elevator position states
    State m_desiredElevatorState;

    // Declares variables for pathing
    Pathing m_pather;

    AutoCommands m_autoCommands;
	// Solenoid innerRingLight;
	// Solenoid outerRingLight;

    /**
     * Constructor to initialize all the teleop commands with the subsystems of the robot and user input devices
     * @param pilotController The controller the pilot uses (Xbox)
     * @param copilotController The controller the copilot uses (gamepad)
     * @param drivetrain The robot's drivetrain subsystem
     * @param elevator The robot's elevator subsystem
     * @param frontClimber The robot's front climber subsystem
     * @param backClimber The robot's back climber subsystem
     * @param hatchMech The robot's hathcmech subsystem
     * @param climberPID The climber PID for the climber subsystems
     * @param pather The pathing to autonomously control the subsystems at the driver's will
     */
    public TeleopCommands(Controller pilotController, GamePad copilotController, Drivetrain drivetrain, Elevator elevator, Climber frontClimber, DriveClimber backClimber, HatchMech hatchMech, ClimberPIDControl climberPID, Pathing pather, AutoCommands autoCommands) {
        m_controller = pilotController;
        m_gamepad = copilotController;
        m_drivetrain = drivetrain;
        m_elevator = elevator;
        m_frontClimber = frontClimber;
        m_backClimber = backClimber;
        m_hatchMech = hatchMech;
        m_climberPID = climberPID;
        m_pather = pather;
        m_autoCommands = autoCommands;

		// innerRingLight = new Solenoid(20, 0);
		// outerRingLight = new Solenoid(20, 1);

        m_desiredElevatorState = State.LEVEL_ZERO;
    }

    /**
     * Designed to be called within TeleopPeriodic in Robot.java This method will go
     * through each method in TeleopCommands class and run any commands that have
     * parameters set to true.
     */
    public void teleopModeCommands() {
        if (m_controller.getBackButton()) {
            // m_autoCommands.pickupAssist();
        }
        else if (m_gamepad.getLevelZero()) {
            m_pather.driveToTarget(0);
        }
        else if (m_gamepad.getLevelZeroReleased()) {
            m_autoCommands.outerRingLight.set(false);
            m_autoCommands.innerRingLight.set(false);
        }
        else if (m_controller.getBackButtonReleased()) {
            m_autoCommands.outerRingLight.set(false);
            m_autoCommands.innerRingLight.set(false);
        }
        else {
            m_autoCommands.resetFlags();
            m_pather.resetFlags();
            controlDrivetrain();
            controlElevator();
            controlHatchMech();
            controlClimbers();
        }
    }

    /**
     * Allows the drivers to control the drivetrain
     */
    public void controlDrivetrain() {
        m_drivetrain.talonArcadeDrive((m_controller.getTriggerAxis(Hand.kRight) - m_controller.getTriggerAxis(Hand.kLeft)), m_controller.getX(Hand.kLeft), true);
    }

    /**
     * Allows the drivers to control the elevator
     */
    public void controlElevator() {
        if (m_gamepad.isManual()) {
            if (m_gamepad.getRawAxis(1) == 1) {
                m_elevator.moveRaw(RobotMap.ELEVATOR_MOTOR_SPEED_UP);
            }
            else if (m_gamepad.getRawAxis(1) == -1) {
                m_elevator.moveRaw(RobotMap.ELEVATOR_MOTOR_SPEED_DOWN);
            }
            else {
                if (m_gamepad.getPickup1Button()) {
                    m_elevator.drivePID(State.HATCH_PICKUP);
                }
                // Button used for automatic dropoff
                // else if (m_gamepad.getLevelZero()) {
                //     m_elevator.drivePID(State.LEVEL_ZERO);    
                // }
                else if (m_gamepad.getPickupHatchCargo()) {
                    m_elevator.drivePID(State.HATCH_PICKUP_2);
                    m_hatchMech.m_servo.setPosition(RobotMap.HATCH_MECH_DIAGONAL_SERVO_POSITION);
                }
                else if (m_gamepad.getLowHatchCargo()) {
                    m_elevator.drivePID(State.HATCH_L1);
                }
                else if (m_gamepad.getMediumHatchCargo()) {
                    m_elevator.drivePID(State.HATCH_L2);
                }
                else if (m_gamepad.getHighHatchCargo()) {
                    m_elevator.drivePID(State.HATCH_L3);
                }
                else {
                    m_elevator.moveRaw(0.0);
                }
            }
        }
        else {
            if (m_gamepad.getLowHatchCargo()) {
                m_elevator.drivePID(State.HATCH_L1);
                m_pather.driveToTarget(6);    
			    // innerRingLight.set(true);
			    // outerRingLight.set(true);
            }
            else if (m_gamepad.getMediumHatchCargo()) {
                m_elevator.drivePID(State.HATCH_L2);
                m_pather.driveToTarget(6);
                // innerRingLight.set(true);
			    // outerRingLight.set(true);
            }
            else if (m_gamepad.getHighHatchCargo()) {
                m_elevator.drivePID(State.HATCH_L3);
                m_pather.driveToTarget(6);
                // innerRingLight.set(true);
			    // outerRingLight.set(true);
            }
            else if (m_gamepad.getPickup1Button()) {
                m_elevator.drivePID(State.HATCH_PICKUP);
            }
            // Button used for automatic dropoff
            // else if (m_gamepad.getLevelZero()) {
            //     m_elevator.drivePID(State.LEVEL_ZERO);    
            // }
            else if (m_gamepad.getPickupHatchCargo()) {
                m_elevator.drivePID(State.HATCH_PICKUP_2);
                m_hatchMech.m_servo.setPosition(RobotMap.HATCH_MECH_DIAGONAL_SERVO_POSITION);
            }
            else {
                controlDrivetrain();
                m_elevator.moveRaw(0.0);
                m_pather.resetFlags();
                // innerRingLight.set(false);
			    // outerRingLight.set(false);
            }
        }
    }

    /**
     * Allows the drivers to control the hatch mechanism
     */
    public void controlHatchMech() {
        if (m_gamepad.getLiftHatchArm()) {
            m_hatchMech.armUp();
        }
        else if (m_gamepad.getDropHatchArm()) {
            m_hatchMech.armDown();
        } 
        else {
            m_hatchMech.setArm(0.0);
        }

        if (m_gamepad.getOpenHatchReleased()) {
            m_hatchMech.openServo();
        }
        else if (m_gamepad.getCloseHatchReleased()) {
            m_hatchMech.closeServo();
        }
    }

    /**
     * Allows the drivers to control the climbers as a pair and seperately
     */
    public void controlClimbers() {
        // PID climber controls bound to pilot controller
		if (m_controller.getAButton()) {
            m_climberPID.climberBangBang();
            m_drivetrain.talonArcadeDrive(0, 0, false);
            m_driveClimberDeployed = true;
		}
        else {
            if (m_controller.getBButton()) {
                m_frontClimber.setClimberSpeed(RobotMap.FRONT_CLIMBER_SPEED_UP);
                m_drivetrain.talonArcadeDrive(0, 0, false);
		    }
            else if (m_controller.getBumper(Hand.kLeft)) {
                m_frontClimber.setClimberSpeed(RobotMap.FRONT_CLIMBER_SPEED_DOWN);
                m_drivetrain.talonArcadeDrive(0, 0, false);
            }
            else {
                m_frontClimber.setClimberSpeed(0);
            }
            
            if (m_controller.getXButton()) {
                m_backClimber.setClimberSpeed(RobotMap.BACK_CLIMBER_SPEED_UP);
                m_drivetrain.talonArcadeDrive(0, 0, false);
                m_driveClimberDeployed = false;
            }
            else if (m_controller.getStartButton()) {
                m_backClimber.setClimberSpeed(RobotMap.BACK_CLIMBER_SPEED_UP_FAST);
                m_drivetrain.talonArcadeDrive(0, 0, false);
                m_driveClimberDeployed = true;
            }
            else if (m_controller.getBumper(Hand.kRight)) {
                m_backClimber.setClimberSpeed(RobotMap.BACK_CLIMBER_SPEED_DOWN);
                m_drivetrain.talonArcadeDrive(0, 0, false);
                m_driveClimberDeployed = true;
            }
            else {
                m_backClimber.setClimberSpeed(0);
            }
        
            // Back climber only can be turned on if the Y Button is held AND the climber is not retracted in the robot
		    if (m_controller.getYButton() ) {
                m_backClimber.driveForward();
                m_drivetrain.talonArcadeDrive(.2, 0, false);
            }
            else {
                m_backClimber.m_driveMotor.set(0);
            }
        }
    }
}