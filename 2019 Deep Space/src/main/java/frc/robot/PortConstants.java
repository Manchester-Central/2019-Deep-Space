/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot; 

/**
 * Assigns port constants
 */
public class PortConstants {


    // test robot ports
    public final static int LEFT_FRONT_SPARK = 3;
	public final static int RIGHT_FRONT_SPARK = 4; 
	public final static int GRABBER_SPARK = 0;
	public final static int FORWARD_HATCH = 0;
	public final static int REVERSE_HATCH = 0;

	static int LEFT_BACK_TALON = 0;
	static int LEFT_MID_TALON = 1;
	static int LEFT_FRONT_TALON = 2;
	
	static int RIGHT_BACK_TALON = 3;
	static int RIGHT_MID_TALON = 4;
	static int RIGHT_FRONT_TALON = 5;
	
	
	static int LEFT_CAN_TALON = 4;
	static int RIGHT_CAN_TALON = 3;

	// real robot ports
	public final static int ELBOW_JOINT = 0;
	public final static int EXTENDER = 0;

	public final static int INTAKE_0 = 0;
	public final static int INTAKE_1 = 0;

	public final static int FLYWHEEL = 0;

	public static final int Potentiometer = 0;

	public static final int EXTENDER_ENCODER_A = 0;
	public static final int EXTENDER_ENCODER_B = 0;
}
