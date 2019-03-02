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

    Controller m_pilotController;
    GamePad m_copilotController;

    Drivetrain m_drivetrain;

    Elevator m_elevator;

    Climber m_frontClimber;
    Climber m_backClimber;

    HatchMech m_hatchMech;

    State m_desiredElevatorState;

    public TeleopCommands(Controller pilotController, GamePad copilotController, Drivetrain drivetrain,
            Elevator elevator, Climber frontClimber, Climber backClimber, HatchMech hatchMech) {
        m_pilotController = pilotController;
        m_copilotController = copilotController;
        m_drivetrain = drivetrain;
        m_elevator = elevator;
        m_frontClimber = frontClimber;
        m_backClimber = backClimber;
        m_hatchMech = hatchMech;
    }

    /**
     * Designed to be called within TeleopPeriodic in Robot.java This method will go
     * through each method in TeleopCommands class and run any commands that have
     * parameters set to true.
     */
    public void teleopModeCommands() {
        controlDrivetrain();
        controlElevator();
        controlHatchMech();
        controlClimbers();
    }

    /**
     * Allows the drivers to control the drivetrain
     */
    public void controlDrivetrain() {
        m_drivetrain.talonArcadeDrive(
                (m_pilotController.getTriggerAxis(Hand.kRight) - m_pilotController.getTriggerAxis(Hand.kLeft)),
                m_pilotController.getX(Hand.kLeft));
    }

    /**
     * Allows the drivers to control the elevator
     */
    public void controlElevator() {

        if (m_copilotController.getManualElevatorUp()) {
            m_elevator.moveRaw(RobotMap.ELEVATOR_MOTOR_SPEED_UP);
        }
        else if (m_copilotController.getManulaElevatorDown()) {
            m_elevator.moveRaw(RobotMap.ELEVATOR_MOTOR_SPEED_DOWN);
        }
        else {
            if (m_copilotController.getPickupHatchCargo()) {
                m_desiredElevatorState = State.LEVEL_ZERO;
            }
            else if (m_copilotController.getLowHatchCargo()) {
                m_desiredElevatorState = State.HATCH_L1;
            }
            else if (m_copilotController.getMediumHatchCargo()) {
                m_desiredElevatorState = State.HATCH_L2;
            }
            else if (m_copilotController.getHighHatchCargo()) {
                m_desiredElevatorState = State.HATCH_L3;
            }

            m_elevator.elevatorPIDDrive(m_desiredElevatorState);
        }

    }

    /**
     * Allows the drivers to control the hatch mechanism
     */
    public void controlHatchMech() {
        if (m_copilotController.getLiftHatchArm()) {
            m_hatchMech.armUp();
        } else if (m_copilotController.getDropHatchArm()) {
            m_hatchMech.armDown();
        } else {
            m_hatchMech.setArm(0.0);
        }

        if (m_copilotController.getOpenHatchReleased()) {
            m_hatchMech.openServo();
        } else if (m_copilotController.getCloseHatchReleased()) {
            m_hatchMech.closeServo();
        }
    }

    /**
     * Allows the drivers to control the climbers as a pair and seperately
     */
    public void controlClimbers() {
        // Raises both climbers
        if (m_pilotController.getBackButton()) {
            m_frontClimber.raiseClimber(RobotMap.FRONT_CLIMBER_SPEED_UP);
            m_backClimber.raiseClimber(RobotMap.BACK_CLIMBER_SPEED_UP);
        }
        // Lowers both climbers
        else if (m_pilotController.getStartButton()) {
            m_frontClimber.lowerClimber(RobotMap.FRONT_CLIMBER_SPEED_DOWN);
            m_backClimber.lowerClimber(RobotMap.BACK_CLIMBER_SPEED_DOWN);
        } else {
            // Raises front climber
            if (m_pilotController.getAButton()) {
                m_frontClimber.raiseClimber(RobotMap.FRONT_CLIMBER_SPEED_UP);
            }
            // Lowers front climber
            else if (m_pilotController.getBButton()) {
                m_frontClimber.lowerClimber(RobotMap.FRONT_CLIMBER_SPEED_DOWN);
            }
            // Defaults front climber motor speed to 0 if no buttons are held
            else {
                m_frontClimber.setClimber(0.0);
            }

            // Raises back climber
            if (m_pilotController.getXButton()) {
                m_backClimber.raiseClimber(RobotMap.BACK_CLIMBER_SPEED_UP);
            }
            // Lowers back climber
            else if (m_pilotController.getYButton()) {
                m_backClimber.lowerClimber(RobotMap.BACK_CLIMBER_SPEED_DOWN);
            }
            // Defaults back climber motor speed to 0 if no buttons are held
            else {
                m_backClimber.setClimber(0.0);
            }
        }

    }
}