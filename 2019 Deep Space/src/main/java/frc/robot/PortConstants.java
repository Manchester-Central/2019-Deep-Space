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

	public final static int Potentiometer = 0;
}
