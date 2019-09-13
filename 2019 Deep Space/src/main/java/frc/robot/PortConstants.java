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
 	// raft robot connections
	static int LEFT_BACK_TALON = 0;
	static int LEFT_MID_TALON = 1;
	static int LEFT_FRONT_TALON = 2;

	static int RIGHT_BACK_TALON = 3;
	static int RIGHT_MID_TALON = 4;
	static int RIGHT_FRONT_TALON = 5;
	
	static int LEFT_CAN_TALON = 4;
	static int RIGHT_CAN_TALON = 3;

	// 2019 robot connections
	public final static int LEFT_FRONT_SPARK = 10; // CAN bus
	public final static int LEFT_BACK_SPARK = 11; // CAN bus
	public final static int RIGHT_FRONT_SPARK = 9; // CAN bus
	public final static int RIGHT_BACK_SPARK = 8; // CAN bus

	public final static int GRABBER_SPARK = 0; // PWM
	public final static int FORWARD_HATCH = 0; // PCM
	public final static int REVERSE_HATCH = 1; // PCM

	public final static int ELBOW_TALON = 3; // CAN bus
	public final static int ELBOW_SPARK = 7; // CAN bus
	public final static int EXTENDER = 4; // CAN bus
	public final static int WRIST = 2; // CAN bus

	public final static int INTAKE_TALON = 1; // CAN bus
	public final static int INTAKE_VICTOR = 6; // CAN bus

	public final static int FLYWHEEL = 5; // CAN bus

	public final static int WRIST_POT = 3; // A2D
	public final static int EXTENDER_POT = 2; // A2D
	public final static int ELBOW_POT = 1; // A2D
	public final static int CLIMBER_POT = 0; // A2D

	public static final int LIMIT_SWITCH_LEFT = 1; // DIO
	public static final int LIMIT_SWITCH_RIGHT = 2; // DIO
	public static final int BEAM_SENSOR = 0; // DIO

	public static final int CLIMB_SOLENOID = 2; // PCM
}
