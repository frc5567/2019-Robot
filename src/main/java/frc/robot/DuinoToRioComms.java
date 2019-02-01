package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

public class DuinoToRioComms {

    private SerialPort duinoPort;

    public DuinoToRioComms() {
        duinoPort = new SerialPort(9600, SerialPort.Port.kUSB);
        System.out.println("Exit Constructor");
    }

    public String pixyRead() {
        System.out.println("Enter pixyRead");
        String sPixyOut = duinoPort.readString();
        System.out.println("Conclude Read");
        System.out.println(sPixyOut);
        System.out.println("Exit Read");
        return sPixyOut;
      //  return Double.parseDouble(sPixyOut);
    }

}