package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

public class NavX extends AHRS {

    // Boolean to keep track of status of offset application
    private boolean m_offsetApplied;

    /**
     * Constructor for NavX with SPI port parameter
     * @param spi_port_id SPI port NavX is connected to
     */
    public NavX(SPI.Port spi_port_id) {
        super(spi_port_id);
        m_offsetApplied = false;
    }

    /**
     * Constructor for NavX with I2C port parameter
     * @param i2c_port_id I2C port NavX is connected to
     */
    public NavX(I2C.Port i2c_port_id) {
        super(i2c_port_id);
        m_offsetApplied = false;
    }

    /**
     * Constructor for NavX with Serial port parameter
     * @param serial_port_id Serial port NavX is connected to
     */
    public NavX(SerialPort.Port serial_port_id) {
        super(serial_port_id);
        m_offsetApplied = false;
    }

    /**
     * Returns the current Yaw reading from the NavX, with offset applied if enabled
     * @return The yaw reading from the NavX
     */
    public float getOffsetYaw() {
        // Checks if offset is enabled
        if (m_offsetApplied) {
            // Adds offset if enabled
            return getYaw() + RobotMap.ANGLE_OFFSET;
        }
        else {
            // Returns yaw without offset applied
            return getYaw();
        }
    }

    /**
     * When called, flips the offset to enabled if disabled and vice versa
     */
    public void flipOffset() {
        // If enabled, disable
        if (m_offsetApplied) {
            m_offsetApplied = false;
        }
        // If disabled, enable
        else {
            m_offsetApplied = true;
        }
    }

    /**
     * Gets the boolean stating whether or not the angle offset is being applied.
     * @return The current status on if the angle offset is being applied
     */
    public boolean getOffsetStatus() {
        return m_offsetApplied;
    }
}