package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import java.nio.ByteBuffer;

public class DuinoToRioComms {
    //  Declaration for usb port to interact with the Duino
    private SerialPort m_duinoPort;

    /**
     *  Constructor for the commuication class object
     */
    public DuinoToRioComms() {
        //  Instantiation for the usb port to interact with the Duino
        m_duinoPort = new SerialPort(9600, SerialPort.Port.kUSB);

        //  Telemetry for testing communication: Print to ensure instantiation
        System.out.println("Exit Constructor");
    }
    
    /**
     * Finds the degrees to target based on the high camera
     * @return  Degrees to target. Will return Double.NaN if it fails to read properly.
     */
    public double getDegToTarget() {
        Double m_degToTarget = Double.NaN;

        //  
        m_degToTarget = pixyRead(2);

        return m_degToTarget;
    }

    /**
     * Finds the distance to target (in) based on the high camera
     * @return  Distance to target (in). Will return Double.NaN if it fails to read properly.
     */
    public double getDistToTarget() {
        Double m_distToTarget = Double.NaN;

        //  
        m_distToTarget = pixyRead(1);

        return m_distToTarget;
    }

    /**
     * Method containing both communication methods
     * @param command The value of the command requested, where 0 requests degreesToTarget, 1 requests dist to target
     */
    private double pixyRead(int command) {
        
        Double m_dataReturned = Double.NaN;

        //  Telemetry for testing communication: Print on enter to check for run
        System.out.println("Enter pixyRead");

        //  Call the data methods with a command inputed in the Robot class
        sendCommand(command);
        m_dataReturned = readData(command);

        if(m_dataReturned.isNaN()){
            System.out.println("Nothing Returned");
        }

        //  Telemetry for testing communication: Print for ensuring the method exits
        System.out.println("Exit Read");

        return m_dataReturned;
    }

    /**
     * Method for sending command to the arduino
     * @param command Command to send to the arduino, where 2 requests degreesToTarget, 1 requests distToTarget
     */
    private void sendCommand(int command) {
        //  Convert the command into a byte array for transmission
        byte[] m_commandByte = ByteBuffer.allocate(4).putInt(command).array();

        //  Write the command down the wire
        m_duinoPort.write(m_commandByte, 4);
    }

    /**
     * Method for receiving data passed back by the arduino
     * @param command Command previously sent to the arduino, where 2 requests degreesToTarget, 1 requests distToTarget
     * @return A double parsed from the string passed by the Duino
     */
    private double readData(int command) {
        Double m_dataDouble = Double.NaN;

        //  Allocates recieved data to a string
        String m_sPixyOut = m_duinoPort.readString();

        if ( !(command == 2 || command == 1) ){
            System.out.println("Invalid Command");
            m_dataDouble = Double.NaN;
        }
        else {
            //  Parses and returns the double sent by the arduino
            m_dataDouble =  Double.parseDouble(m_sPixyOut);
        }

        return m_dataDouble;
        
    }
}