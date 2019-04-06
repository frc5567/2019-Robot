package frc.robot;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Defines the mechanism that picks up cargo to put it in the cargo ships and rockets.
 */
public class BallGrabber {



//  Declararations for both arm motors for the BallGrabber

SpeedController m_rightArmGrabberMotor;
SpeedController m_leftArmGrabberMotor;
SpeedControllerGroup m_grabberArmMotors;
SpeedController m_rightRotateGrabberMotor;
SpeedController m_leftRotateGrabberMotor;
SpeedControllerGroup m_grabberRotateMotors;

Encoder grabberRotate;

void GrabberArm(int leftChannel, int rightChannel){
    //  Instantiates grabber arm motors. Change Spark if different speed controller is used.
    m_leftArmGrabberMotor = new Spark(leftChannel);
    m_rightArmGrabberMotor = new Spark(rightChannel);

    m_grabberArmMotors = new SpeedControllerGroup(m_leftArmGrabberMotor, m_rightArmGrabberMotor);

}
void GrabberRotate(int leftChannel, int rightChannel){
    //  Instantiates grabber rotate motors. Change Spark if different speed controller is used.
    m_leftRotateGrabberMotor = new Spark(leftChannel);
    m_rightRotateGrabberMotor = new Spark(rightChannel);

    m_grabberRotateMotors = new SpeedControllerGroup(m_leftRotateGrabberMotor, m_rightRotateGrabberMotor);


}
    //  Constructor for instantiating Grabber Encoder
void GrabberEncoder(int leftChannel, int rightChannel){

    grabberRotate = new Encoder(leftChannel, rightChannel, false, Encoder.EncodingType.k1X);
}







}