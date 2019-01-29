/**
 * This class defines what a color sensor is and how it functions.
 */

package frc.robot;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;

public class ColorSensor {
    int capacity; // Needs to have a value.
    ByteBuffer buffer = java.nio.ByteBuffer.allocate(capacity);
    I2C sensor;
    public ColorSensor(int address){
        sensor = new I2C(I2C.Port.kOnboard, address);
        sensor.write(0x00, 192); //b11000000 ... Power on, color sensor on. (page 20 of sensor datasheet)
    }
    public int red(){
        sensor.read(0x16, 1, buffer);
        return buffer.get(0);
    }
    public int green(){
        sensor.read(0x18, 1, buffer);
        return buffer.get(0);
    }
    public int blue(){
        sensor.read(0x1A, 1, buffer);
        return buffer.get(0);
    }

}