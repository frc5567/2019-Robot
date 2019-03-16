package frc.robot;

import frc.robot.Elevator.State;

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

    // boolean mainStages[];
    // boolean stage1Steps[];
    // boolean stage2Steps[];
    // boolean stage3Steps[];
    // boolean stage4Steps[];

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

        // mainStages = new boolean[] { false, false, false, false };
        // stage1Steps = new boolean[] { false, false, false, false };
        // stage2Steps = new boolean[] { false, false, false, false, false, false, false };
        // stage3Steps = new boolean[] { false, false, false };
        // stage4Steps = new boolean[] { false, false, false };

        stage1 = false;
        stage2 = false;
        stage3 = false;
        stage4 = false;
        partOneAssist = false;
        partTwoAssist = false;
        partThreeAssist = false;

    }

/*
    public void AutoDrive() {
        if (!mainStages[0]) {
            if (!stage1Steps[0]) {
                stage1Steps[0] = driveStraight(-0);
            }
            else if (!stage1Steps[1]) {
                stage1Steps[1] = driveCurve(-0, 0);
            }
            else if (!stage1Steps[2]) {
                stage1Steps[2] = driveRotate(0);
            }
            else if (!stage1Steps[3]) {
                // Vision pathing to place hatch
            }
            else {
                mainStages[0] = true;
            }
        }
        else if (!mainStages[1]) {
            if (!stage2Steps[0]) {
                stage2Steps[0] = driveStraight(-0);
            }
            else if (!stage2Steps[1]) {
                stage2Steps[1] = driveRotate(0);
            }
            else if (!stage2Steps[2]) {
                stage2Steps[2] = driveStraight(+0);
            }
            else if (!stage2Steps[3]) {
                stage2Steps[3] = driveCurve(+0, 0);
            }
            else if (!stage2Steps[4]) {
                stage2Steps[4] = driveCurve(+0, 0);
            }
            else if (!stage2Steps[5]) {
                stage2Steps[5] = driveStraight(+0);
            }
            else if (!stage2Steps[6]) {
                // Vision pathing to retrieve hatch
            }
            else {
                mainStages[1] = true;
            }
        }
        else if (!mainStages[2]) {
            if (!stage3Steps[0]) {
                stage3Steps[0] = driveStraight(-0);
            } 
            else if (!stage3Steps[1]) {
                stage3Steps[1] = driveRotate(0);
            }
            else if (!stage3Steps[2]) {
                // Vision pathing to place hatch
            }
            else {
                mainStages[2] = true;
            }
        } else if (!mainStages[3]) {
            if (!stage4Steps[0]) {
                stage4Steps[0] = driveStraight(-0);
            }
            else if (!stage4Steps[1]) {
                stage4Steps[1] = driveRotate(0);
            }
            else if (!stage4Steps[2]) {
                stage4Steps[2] = driveStraight(+0);
            }
            else {
                mainStages[3] = true;
            }
        } else {

        }
    }
*/

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

    /**
     * This method is for driving the robot at an angle
     * 
     * @param distance The distance the robot will travel
     * @param angle    The angle at which the robot will travel
     * @return Returns true if finished, false if it is not finished
     */
/*
    public boolean driveCurve(int distance, int angle) {
        return false;
    }
*/

    public void resetFlags() {
        stage1 = false;
        stage2 = false;
        stage3 = false;
        stage4 = false;
        partOneAssist = false;
        partTwoAssist = false;
        partThreeAssist = false;
    }

/*
    public void driveStraight(int distance, int angle) {

    }
*/

    /**
     * Rotates the robot in place
     * 
     * @param angle The angle the robot will rotate to
     * @return Returns true if finished, false if it is not finished
     */
/*
    public boolean driveRotate(int angle) {
        return m_drivetrain.rotateToAngle(angle);
    }
*/

    /**
     * Drives the robot straight for a specified distance
     * 
     * @param distance The distance the robot will travel
     * @return Returns true if finished, false if it is not finished
     */
/*
    public boolean driveStraight(int distance) {
        return false;
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
*/
}