/*Defines Mechanism that allows hatches to be placed on cargo ships and rockets.
*/
package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.VictorSP;          //Might change VictorSP for motor later
import edu.wpi.first.wpilibj.DigitalInput;

public class HatchMech{

    Servo m_servo;

    VictorSP m_hatchArmMotor;
    DigitalInput m_limitSwitchTop;
    DigitalInput m_limitSwitchBottom;

        
    HatchMech(){
        //create servo variable for hatch mechanism
        m_servo = new Servo(1);         //need to add the channel that the servo will be plugged into

        m_limitSwitchTop = new DigitalInput(0);
        m_limitSwitchBottom = new DigitalInput(1);
    }

    public void OpenServo(){
        m_servo.setPosition(0.6);            //double check on degrees and see if servo is the right type
    }

    public void CloseServo(){
        m_servo.setPosition(0.3);
    }

    public void SwitchServo(boolean button){
        if (button){
            if (m_servo.getAngle()==0.6){
                CloseServo();
            }
            else{
                OpenServo();
            }
        }
    }

    public void ArmUp(){
        while(m_limitSwitchTop.get() == false){
            m_hatchArmMotor.set(0.5);                   // Double check direction and speed
        }
        m_hatchArmMotor.set(0.0);
                               
    }

    public void ArmDown(){
        while(m_limitSwitchBottom.get() == false){
        m_hatchArmMotor.set(-0.5);                       // Double check direction and speed        
        }
        m_hatchArmMotor.set(0.0);
    }


}


// If the servo uses 0.0 to 1.0 use - exampleServo.set(.5);
// use this for 0 to 180 degrees - exampleServo.setAngle(75);