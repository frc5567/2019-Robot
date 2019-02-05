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
        //  Declares and instantiates a value for storing the return from pixyRead
        Double degToTarget = Double.NaN;

        //  Calls pixyRead with the command 2 to get deg to target and assign it to return variable
        degToTarget = pixyRead('2');

        return degToTarget;
    }

    /**
     * Finds the distance to target (in) based on the high camera
     * @return  Distance to target (in). Will return Double.NaN if it fails to read properly.
     */
    public double getDistToTarget() {
        //  Declares and instantiates a value for storing the return from pixyRead
        Double distToTarget = Double.NaN;

        //  Calls pixyRead with the command 1 to get dist to target and assign it to return variable
        distToTarget = pixyRead('1');

        return distToTarget;
    }

    /**
     * Method containing both communication methods
     * @param command The value of the command requested, where 0 requests degreesToTarget, 1 requests dist to target
     */
    private double pixyRead(char command) {
        //  Declares and instantiates a variable for storing return from readData        
        Double dataReturned = Double.NaN;

        //  Telemetry for testing communication: Print on enter to check for run
        System.out.println("Enter pixyRead");

        //  Call the data methods with a command inputed in the Robot class
        sendCommand(command);
        dataReturned = readData(command);

        //  Telemetry for checking if the returned data was valid
        if(dataReturned.isNaN()){
            System.out.println("Nothing Returned");
        }

        //  Telemetry for testing communication: Print for ensuring the method exits
        System.out.println("Exit Read");

        return dataReturned;
    }

    /**
     * Method for sending command to the arduino
     * @param command Command to send to the arduino, where 2 requests degreesToTarget, 1 requests distToTarget
     */
    private void sendCommand(char command) {
        //  Convert the command into a byte array for transmission
        byte commandByte = (byte)command;
        byte[] commandStorage = new byte[1];
        commandStorage[0] = commandByte;

        //  Write the command down the wire
        m_duinoPort.write(commandStorage, 1);
    }

    /**
     * Method for receiving data passed back by the arduino
     * @param command Command previously sent to the arduino, where 2 requests degreesToTarget, 1 requests distToTarget
     * @return A double parsed from the string passed by the Duino
     */
    private double readData(char command) {
        //  Declares and instantiates a variable for storing return from arduino        
        Double dataDouble = Double.NaN;

        //  Allocates recieved data to a string
        String sPixyOut = m_duinoPort.readString();

        //  Checks to see if the passed in command is valid
        if ( !(command == '2' || command == '1') ){
            System.out.println("Invalid Command");
        }
        else {
            //  Parses the double sent by the arduino
            try {
                dataDouble =  Double.parseDouble(sPixyOut);
            }
            catch (NumberFormatException e) {
                System.out.println ("No parsable number returned");
            }
            catch (NullPointerException e) {
                System.out.println ("Null Pointer Exception: Nothing passed in");
            }
            catch (Exception e) {
                System.out.println ("Unknown Exception");
            }
        }

        return dataDouble;
        
    }
}