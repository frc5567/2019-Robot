/*Defines Mechanism that allows hatches to be placed on cargo ships and rockets.
*/
package frc.robot;

//Imports servo, motor controller, and digital input (true or false) for later use.
import edu.wpi.first.wpilibj.Servo;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class HatchMech{

    //Initializes the servo so it can be used for position of the linear actuator.
    Servo m_servo;  

    //Initializes the victor, and the limit switches for use moving the hatch mechanism arm.
    WPI_VictorSPX m_hatchArmMotor;
    DigitalInput m_limitSwitchTop;
    DigitalInput m_limitSwitchBottom;
    Encoder m_hatchMechEncoder;
  
    public HatchMech(){
        //Creates servo variable for hatch mechanism.
        //Need to add the channel that the servo will be plugged into.
        m_servo = new Servo(RobotMap.HATCH_MECH_SERVO_PORT);

        //This defines the limit switches as new digital inputs.
        m_limitSwitchTop = new DigitalInput(RobotMap.HATCH_MECH_LIMIT_TOP_PORT);
        m_limitSwitchBottom = new DigitalInput(RobotMap.HATCH_MECH_LIMIT_BOTTOM_PORT);

        m_hatchMechEncoder = new Encoder(RobotMap.HATCH_MECH_ENCODER_A, RobotMap.HATCH_MECH_ENCODER_B, false, Encoder.EncodingType.k4X);
        m_hatchMechEncoder.reset();
    }

    /**
     * Creates method for opening servo, and sets relative posisition.
     */
    public void OpenServo(){
        //Double check on degrees and see if servo is the right type.
        m_servo.setPosition(RobotMap.HATCH_MECH_OPEN_SERVO_POSITION);
    }

    /**
     * Creates method for closing servo and sets relative position.
     *  */
    public void CloseServo(){
        m_servo.setPosition(RobotMap.HATCH_MECH_CLOSE_SERVO_POSITION);
    }
 
    /**
     * Creates method for switching between closed and opened servo. To do this a boolean is used to make a toggle button.
     * @param button
     */
    public void SwitchServo(boolean button){
        if (button){
            if (m_servo.getAngle()==RobotMap.HATCH_MECH_OPEN_SERVO_POSITION){   
                CloseServo();
            }
            else{
                OpenServo();
            }
        }
    }

    /**
     * Raises the Hatch Mech arm to its high position when called
     */
    public void ArmUp(){
        if (m_hatchMechEncoder.get() >= RobotMap.HATCH_MECH_UP_MOTOR_POSITION) {
            m_hatchArmMotor.set(RobotMap.HATCH_MECH_ARM_UP_MOTOR_SPEED);
        }
        else {
            m_hatchArmMotor.set(RobotMap.HATCH_MECH_STOP_MOTOR_SPEED);
        }
                               
    }
   
    /**
     * Lowers the Hatch Mech arm to its low arm when called
     */
    public void ArmDown(){
        if (m_hatchMechEncoder.get() <= RobotMap.HATCH_MECH_DOWN_MOTOR_POSITION) {
            m_hatchArmMotor.set(RobotMap.HATCH_MECH_ARM_DOWN_MOTOR_SPEED);                            
        }
        else {
            m_hatchArmMotor.set(RobotMap.HATCH_MECH_STOP_MOTOR_SPEED);
        }
    }
}
