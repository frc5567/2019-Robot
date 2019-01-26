package frc.robot;

import edu.wpi.first.wpilibj.SerialPort;

public class DuinoToRioComms {

    private SerialPort duinoPort;

    public DuinoToRioComms() {
        duinoPort = new SerialPort(9600, SerialPort.Port.kUSB);
    }

    public double pixyRead() {
        return Double.parseDouble(duinoPort.readString());
    }

}