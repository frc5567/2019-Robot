package frc.robot;

import frc.robot.Elevator.State;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * This class is to be able to command the robot via autonomous and auto assist
 * using emthods within this class. The methods can combined together to form
 * full pathing
 * 
 * @author Matt
 * @version Week 5 Pre-comp
 */
public class AutoCommands {

    Drivetrain m_drivetrain;
    NavX m_gyro;
    Elevator m_elevator;
    Climber m_frontClimber;
    Climber m_backClimber;
    Pathing m_pathing;
    TeleopCommands m_teleopCommands;
    HatchMech m_hatchMech;

    boolean stage1;
    boolean stage2;
    boolean stage3;
    boolean stage4;

    boolean partOneAssist;
    boolean partTwoAssist;
    boolean partThreeAssist;

    Solenoid outerRingLight;
    Solenoid innerRingLight;

    public AutoCommands(Drivetrain drivetrain, NavX ahrs, Elevator elevator, Climber frontClimber, Climber backClimber, Pathing pathing, TeleopCommands teleopCommands, HatchMech hatchMech, Solenoid outerLight, Solenoid innerLight) {
        m_drivetrain = drivetrain;
        m_gyro = ahrs;
        m_elevator = elevator;
        m_frontClimber = frontClimber;
        m_backClimber = backClimber;
        m_pathing = pathing;
        m_teleopCommands = teleopCommands;
        m_hatchMech = hatchMech;
        outerRingLight = outerLight;
        innerRingLight = innerLight;
    
        stage1 = false;
        stage2 = false;
        stage3 = false;
        stage4 = false;
        partOneAssist = false;
        partTwoAssist = false;
        partThreeAssist = false;

    }

    public void pickupAssist() {
        if (!partOneAssist) {
		    outerRingLight.set(true);
		    innerRingLight.set(true);
            m_hatchMech.openServo();
            m_elevator.drivePID(State.HATCH_PICKUP);
            partOneAssist = m_pathing.secondHalfPath(12);
        }
        else if (!partTwoAssist) {
            m_hatchMech.closeServo();
            m_elevator.drivePID(State.HATCH_PICKUP_2);
            partTwoAssist = ((m_elevator.m_motor.getSelectedSensorVelocity() < 10) && (m_elevator.getPosition() > (State.HATCH_L1.getDeltaHeight() + 1)));
            outerRingLight.set(false);
            innerRingLight.set(false);
        }
        else if (!partThreeAssist) {
            m_elevator.moveRaw(0);
            // m_hatchMech.armUp();
            m_drivetrain.driveToPositionAngle(-20, 0, 0.2);
        }
        else {
            // m_hatchMech.setArm(0);
            m_drivetrain.talonArcadeDrive(0, 0, false);
        }
    }

    public void resetFlags() {
        stage1 = false;
        stage2 = false;
        stage3 = false;
        stage4 = false;
        partOneAssist = false;
        partTwoAssist = false;
        partThreeAssist = false;
    }
}