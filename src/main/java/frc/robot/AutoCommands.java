package frc.robot;

import frc.robot.Elevator.State;

/**
 * This class is to be able to command the robot via autonomous and auto assist
 * using emthods within this class. The methods can combined together to form full pathing
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

    public AutoCommands(Drivetrain drivetrain, NavX ahrs, Elevator elevator, Climber frontClimber, Climber backClimber, Pathing pathing, TeleopCommands teleopCommands, HatchMech hatchMech) {
        m_drivetrain = drivetrain;
        m_gyro = ahrs;
        m_elevator = elevator;
        m_frontClimber = frontClimber;
        m_backClimber = backClimber;
        m_pathing = pathing;
        m_teleopCommands = teleopCommands;
        m_hatchMech = hatchMech;

        stage1 = false;
        stage2 = false;
        stage3 = false;
        stage4 = false;
    }

    public void AutoDrive() {
        if (!stage1) {

        }
        else if (!stage2) {

        }
        else if (!stage3) {

        }
        else if (!stage4) {

        }
        else {
            
        }
    }

    public void driverAssist() {
        if (!partOneAssist) {
            m_elevator.drivePID(State.HATCH_PICKUP);
            partOneAssist = m_pathing.secondHalfPath(12);
        }
        else if (!partTwoAssist) {
            partTwoAssist = (m_elevator.getPosition() == State.HATCH_PICKUP_2.getDeltaHeight());
            m_elevator.drivePID(State.HATCH_PICKUP_2);
        }
        else if (!partThreeAssist) {
            m_hatchMech.closeServo();
            m_drivetrain.driveToPositionAngle(-20, 0, 0.5);
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

    public void driveStraight(int distance, int angle) {

    }

    public void driveRotate(int angle) {

    }

    public void elevatorMove(Elevator.State state) {

   }

   public void frontClimberRise() {

   }

   public void frontClimberLower() {

   }

   public void backClimberRise() {

   }

   public void backClimberLower() {
       
   }
}