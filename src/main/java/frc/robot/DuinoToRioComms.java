package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

public class DuinoToRioComms {
    //  Declaration for usb port to interact with the Duino
	private SerialPort m_duinoPort;
	private final char GET_DEG_TO_TARGET = '2';
	private final char GET_DIST_TO_TARGET = '1';
	private final char GET_ANGLE_TO_CENTER = '3';
	private final char GET_LOW_POSITION = '4';

    /**
     *  Constructor for the commuication class object
     */
    public DuinoToRioComms() {
        //  Instantiation for the usb port to interact with the Duino
        m_duinoPort = new SerialPort(9600, SerialPort.Port.kUSB);
        m_duinoPort.enableTermination();
    }
    
    /**
     * Finds the degrees to target based on the high camera
     * @return  Degrees to target. Will return Double.NaN if it fails to read properly.
     */
    public double getDegToTarget() {
        //  Declares and instantiates a value for storing the return from pixyRead
        Double degToTarget = Double.NaN;

        //  Calls pixyRead with the command 2 to get deg to target and assign it to return variable
        degToTarget = pixyRead(GET_DEG_TO_TARGET);

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
        distToTarget = pixyRead(GET_DIST_TO_TARGET);

        return distToTarget;
    }

    /**
     * Finds the angle to target based on the low camera
     * @return  Angle to target. Will return Double.NaN if it fails to read properly.
     */
    public double getAngleToCenter() {
        //  Declares and instantiates a value for storing the return from pixyRead
        Double angleToCenter = Double.NaN;

        //  Calls pixyRead with the command 3 to get angle to center and assign it to return variable
        angleToCenter = pixyRead(GET_ANGLE_TO_CENTER);

        return angleToCenter;
    }

    /**
     * Finds the approximate position relative to the target based on the high camera
     * @return  Position relative to target, where -1 is no target, 1 is left, 2 is center, and 3 is right. Will return Double.NaN if it fails to read properly.
     */
    public double getLowPosition() {
        //  Declares and instantiates a value for storing the return from pixyRead
        Double lowPosition = Double.NaN;

        //  Calls pixyRead with the command 4 to get lowPosition and assign it to return variable
        lowPosition = pixyRead(GET_LOW_POSITION);

        return lowPosition;
    }

    /**
     * Method containing both communication methods
     * @param command The value of the command requested, where 0 requests degreesToTarget, 1 requests dist to target
     */
    private double pixyRead(char command) {
        //  Declares and instantiates a variable for storing return from readData        
        Double dataReturned = Double.NaN;

        //  Call the data methods with a command inputed in the Robot class
        sendCommand(command);
        dataReturned = readData(command);

        //  Telemetry for checking if the returned data was valid
        if(dataReturned.isNaN()){
            System.out.println("Nothing Returned");
        }

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
        String sPixyOut = m_duinoPort.readString(6);

        //  Checks to see if the passed in command is valid
        if ( (command != GET_ANGLE_TO_CENTER && command != GET_DEG_TO_TARGET) && (command != GET_DIST_TO_TARGET && command != GET_LOW_POSITION) ){
            System.out.println("Invalid Command");
		}
        else {
            //  If the parseDouble throws an exception, the robot would crash. This catches
            //  those exceptions and prints to tell us why
            try {
                //  Parses the double sent by the arduino
                dataDouble =  Double.parseDouble(sPixyOut);
            }
            catch (NumberFormatException e) {
                //  If the value returned is not parsable, we hit this exception
                System.out.println ("No parsable number returned");
            }
            catch (NullPointerException e) {
                //  If the value returned is null, we hit this exception
                System.out.println ("Null Pointer Exception: Nothing passed in");
            }
            catch (Exception e) {
                //  If we hit this exception, we're in trouble, as it should not be possible for the parse method
                System.out.println ("Unknown Exception");
            }
        }

        return dataDouble;
        
    }
}