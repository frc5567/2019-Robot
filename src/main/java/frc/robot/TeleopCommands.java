package frc.robot;

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

    Controller m_controller;
    GamePad m_gamepad;

    Drivetrain m_drivetrain;

    Elevator m_elevator;

    Climber m_frontClimber;
    DriveClimber m_backClimber;
    ClimberPIDControl m_climberPID;

    HatchMech m_hatchMech;

    boolean m_driveClimberDeployed = false;

    State m_desiredElevatorState;

    public TeleopCommands(Controller pilotController, GamePad copilotController, Drivetrain drivetrain, Elevator elevator, Climber frontClimber, DriveClimber backClimber, HatchMech hatchMech, ClimberPIDControl climberPID) {
        m_controller = pilotController;
        m_gamepad = copilotController;
        m_drivetrain = drivetrain;
        m_elevator = elevator;
        m_frontClimber = frontClimber;
        m_backClimber = backClimber;
        m_hatchMech = hatchMech;
        m_climberPID = climberPID;
    }

    /**
     * Designed to be called within TeleopPeriodic in Robot.java This method will go
     * through each method in TeleopCommands class and run any commands that have
     * parameters set to true.
     */
    public void teleopModeCommands() {
        if (!m_driveClimberDeployed) {
            controlDrivetrain();
        }
        controlElevator();
        controlHatchMech();
        controlClimbers();
    }

    /**
     * Allows the drivers to control the drivetrain
     */
    public void controlDrivetrain() {
        m_drivetrain.talonArcadeDrive((m_controller.getTriggerAxis(Hand.kRight) - m_controller.getTriggerAxis(Hand.kLeft)), m_controller.getX());
    }

    /**
     * Allows the drivers to control the elevator
     */
    public void controlElevator() {

        if (m_gamepad.getRawAxis(1) == -1) {
            m_elevator.moveRaw(RobotMap.ELEVATOR_MOTOR_SPEED_UP);
        }
        else if (m_gamepad.getRawAxis(1) == -1) {
            m_elevator.moveRaw(RobotMap.ELEVATOR_MOTOR_SPEED_DOWN);
        }
        else {
            if (m_gamepad.getPickupHatchCargo()) {
                m_desiredElevatorState = State.LEVEL_ZERO;
            }
            else if (m_gamepad.getLowHatchCargo()) {
                m_desiredElevatorState = State.HATCH_L1;
            }
            else if (m_gamepad.getMediumHatchCargo()) {
                m_desiredElevatorState = State.HATCH_L2;
            }
            else if (m_gamepad.getHighHatchCargo()) {
                m_desiredElevatorState = State.HATCH_L3;
            }

            m_elevator.drivePID(m_desiredElevatorState);
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
        } else {
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
            m_climberPID.climberPIDDrive(RobotMap.CLIMBER_TARGET);
            m_driveClimberDeployed = true;
		}
		else if (m_controller.getBButton()) {
			m_frontClimber.raiseClimber(RobotMap.FRONT_CLIMBER_SPEED_UP);
		}
		else if (m_controller.getXButton()) {
            m_backClimber.raiseClimber(RobotMap.FRONT_CLIMBER_SPEED_UP);
            m_driveClimberDeployed = false;
        }
        else if (m_controller.getBumper(Hand.kLeft)) {
			m_frontClimber.lowerClimber(RobotMap.FRONT_CLIMBER_SPEED_DOWN);
		}
		else if (m_controller.getBumper(Hand.kRight)) {
            m_backClimber.lowerClimber(RobotMap.FRONT_CLIMBER_SPEED_DOWN);
            m_driveClimberDeployed = true;
		}
		else if (m_controller.getYButton() && m_driveClimberDeployed) {
            m_backClimber.driveForward();
            m_drivetrain.talonArcadeDrive(.2, 0);
        }
        
    }
}