/*Defines Mechanism that allows hatches to be placed on cargo ships and rockets.
 */
package frc.robot;

import edu.wpi.first.wpilibj.Servo;

public class HatchMech{

    Servo m_servo;
    

    HatchMech() {
    //create servo variable for hatch mechanism
        m_servo = new Servo(1);             //need to add the channel that the servo will be plugged into
    }

    public void OpenServo(){
         m_servo.setAngle(0);       //double check on degrees and see if servo is the right type
    }

    public void ClosedServo(){
        m_servo.setAngle(90);
    }

    public void switchServo(boolean button){
        if (button){
            if (m_servo.getAngle()==0){
                ClosedServo();
            }
            else{
                OpenServo();
            }
        }
    }

}


// If the servo uses 0.0 to 1.0 use - exampleServo.set(.5);
// use this for 0 to 180 degrees - exampleServo.setAngle(75);