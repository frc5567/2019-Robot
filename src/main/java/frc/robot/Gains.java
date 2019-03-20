package frc.robot;

/**
 *  Class that organizes gains used when assigning values to slots
 *  This is sample code straight ripped from CTRE's examples, so thank you CTRE
 */
public class Gains {
	// Declares the constants used in the PID controller
	public final double kP;
	public final double kI;
	public final double kD;
	public final double kF;
	public final int kIzone;
	public final double kPeakOutput;
	
	/**
	 * The constructor to hold all of the constants for a PID controller
	 * @param _kP The proportional constant (kP)
	 * @param _kI The intergral constant (kI)
	 * @param _kD The derivative constant (kD)
	 * @param _kF The feed-forward constant (kF)
	 * @param _kIzone The intergral zone constant
	 * @param _kPeakOutput The peak output the PID controller can output
	 */
	public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
		kP = _kP;
		kI = _kI;
		kD = _kD;
		kF = _kF;
		kIzone = _kIzone;
		kPeakOutput = _kPeakOutput;
	}
}