/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class ArmConstants {

    public static final double ACCEPTABLE_ANGLE = 4;

    public static final double ELBOW_RADIUS = 4.0;
    public static final double ENCODER_TICKS_PER_REVOLUTION = 4100D;
    public static final double MIN_ELBOW_VOLTAGE = 0;
    public static final double MAX_ELBOW_VOLTAGE = 0;
    public static final double MIN_ELBOW_ANGLE = 0;
    public static final double MAX_ELBOW_ANGLE = 0;
    
    public static final double MIN_EXTENDER_VOLTAGE = 0;
    public static final double MAX_EXTENDER_VOLTAGE = 0;
    public static final double MIN_EXTENDER_LENGTH = 0;
    public static final double MAX_EXTENDER_LENGTH = 0;

    public static final double ARM_DISTANCE = 22.794;
    public static final double ARM_WEIGHT = 20.0;
    public static final double EXTENSION_WEIGHT = 20.0;
    public static final double TOTAL_WEIGHT = ARM_WEIGHT + EXTENSION_WEIGHT;
    public static final double ARM_HEIGHT_INCHES = 46.063;
    public static final double ARM_DISTANCE_X_FROM_FRAME = 5;
    public static final double MAX_REACH_X_BEYOND_FRAME = 30;
    public static final double GRABBER_LENGTH = 12;
    public static final double MAX_REACH_X = ARM_DISTANCE_X_FROM_FRAME + MAX_REACH_X_BEYOND_FRAME - GRABBER_LENGTH;  

    public static final double GEAR_RATIO = 2.0;
    public static final double MOTOR_STALL_TORQUE = 0.71;

    public static final double ELBOW_P = 0.001;
	public static final double ELBOW_I = 0.4;
    public static final double ELBOW_D = 0;
    public static final double EXTENDER_P = 0.001;
    public static final double EXTENDER_I = 0;
    public static final double EXTENDER_D = 0;

    
    public static final double BALL_PICKUP_ANGLE = -90;

    public static final double BALL_LOW = 27.5;
    public static final double BALL_MID = 55.5;
    public static final double BALL_HIGH = 83.5;
    public static final double HATCH_LOW = 19;
    public static final double HATCH_MID = 47;
    public static final double HATCH_HIGH = 75;
    public static final double CARGO_BALL = 37;

}
