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
     * Method containing both communication methods
     * @param command The value of the command requested, where 0 requests degreesToTarget, 1 requests dist to target
     */
    public void pixyRead(int command) {
        //  Telemetry for testing communication: Print on enter to check for run
        System.out.println("Enter pixyRead");

        //  Call the data methods with a command inputed in the Robot class
        sendCommand(command);
        readData(command);

        //  Telemetry for testing communication: Print for ensuring the method exits
        System.out.println("Exit Read");
    }

    /**
     * Method for sending command to the arduino
     * @param command Command to send to the arduino, where 0 requests degreesToTarget, 1 requests distToTarget
     */
    private void sendCommand(int command) {
        //  Convert the command into a byte array for transmission
        byte[] m_commandByte = ByteBuffer.allocate(4).putInt(command).array();

        //  Write the command down the wire
        m_duinoPort.write(m_commandByte, 4);
    }

    /**
     * Method for receiving data passed back by the arduino
     * @param command Command previously sent to the arduino, where 0 requests degreesToTarget, 1 requests distToTarget
     * @return A double parsed from the string passed by the Duino
     */
    private double readData(int command) {
        //  Allocates recieved data to a string
        String m_sPixyOut = m_duinoPort.readString();

        //  Checks command and prints based off of that print
        if (command == 2) {
            System.out.println("degToTarget" + m_sPixyOut);
        } else if (command == 1) {
            System.out.println("distToTarget" + m_sPixyOut);
        } else {
            System.out.println("Invalid Command");
        }

        //  Parses and returns the double sent by the arduino
        return Double.parseDouble(m_sPixyOut);
    }
}