package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;
import java.nio.ByteBuffer;

public class DuinoToRioComms {

    private SerialPort duinoPort;

    public DuinoToRioComms() {
        duinoPort = new SerialPort(9600, SerialPort.Port.kUSB);
        System.out.println("Exit Constructor");
    }

    public void pixyRead(int command) {
        System.out.println("Enter pixyRead");
        sendCommand(command);
        readData(command);
        System.out.println("Conclude Read");
        System.out.println("Exit Read");
    }

    private void sendCommand(int command) {
        byte[] commandByte = ByteBuffer.allocate(4).putInt(command).array();
        duinoPort.write(commandByte, 4);
    }

    private double readData(int command) {
        String sPixyOut = duinoPort.readString();
        if (command == 0) {
            System.out.println("degToTarget" + sPixyOut);
        } else if (command == 1) {
            System.out.println("distToTarget" + sPixyOut);
        } else {
            System.out.println("Invalid Command");
        }
        return Double.parseDouble(sPixyOut);
    }
}