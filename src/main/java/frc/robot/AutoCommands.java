package frc.robot;

/**
 * This class is to be able to command the robot via autonomous and auto assist
 * using emthods within this class. The methods can combined together to form full pathing
 * @author Matt
 * @version Week 5 Pre-comp
 */
public class AutoCommands {

    Drivetrain m_drivetrain;
    NavX m_ahrs;
    Elevator m_elevator;
    Climber m_frontClimber;
    Climber m_backClimber;

    public AutoCommands(Drivetrain drivetrain, NavX ahrs, Elevator elevator, Climber frontClimber, Climber backClimber) {
        m_drivetrain = drivetrain;
        m_ahrs = ahrs;
        m_elevator = elevator;
        m_frontClimber = frontClimber;
        m_backClimber = backClimber;

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