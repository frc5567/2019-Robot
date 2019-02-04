/**
 * This is the mechanism that allows the robot to climb in the end game.
 */
package frc.robot;

//Importing the Victor and Talon speed controller groups
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Lift{

    // This is just declaring the motors for later use
    WPI_TalonSRX m_frontLeftLiftMotor;
    WPI_TalonSRX m_frontRightLiftMotor;
    WPI_VictorSPX m_backLeftLiftMotor;
    WPI_VictorSPX m_backRightLiftMotor;
    
    //This is making left and right controller groups 
    SpeedControllerGroup m_rightLift;
    SpeedControllerGroup m_leftLift; 

    
    Lift(int frontLeft, int frontRight, int backLeft, int backRight){

        // Putting the variables into the constructor which allows for the class to be used 
        m_frontLeftLiftMotor = new WPI_TalonSRX(frontLeft);
        m_frontRightLiftMotor = new WPI_TalonSRX(frontRight);
        m_backLeftLiftMotor = new WPI_VictorSPX(backLeft);
        m_backRightLiftMotor = new WPI_VictorSPX(backRight);

        // Creates a group that includes the front right motor and the back right motors. And another group that includes the front and back left motors.
        m_rightLift = new SpeedControllerGroup(m_frontRightLiftMotor, m_backRightLiftMotor);
        m_leftLift = new SpeedControllerGroup(m_frontLeftLiftMotor, m_backLeftLiftMotor);

    }
}