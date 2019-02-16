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

    public static final double ARM_DISTANCE = 20.0;
    public static final double ARM_WEIGHT = 20.0;
    public static final double EXTENSION_WEIGHT = 20.0;
    public static final double TOTAL_WEIGHT = ARM_WEIGHT + EXTENSION_WEIGHT;
    public static final double ARM_HEIGHT_INCHES = 42;
    public static final double ARM_DISTANCE_X_FROM_FRAME = 5;
    public static final double MAX_REACH_X_BEYOND_FRAME = 30;
    public static final double MAX_REACH_X = ARM_DISTANCE_X_FROM_FRAME + MAX_REACH_X_BEYOND_FRAME;  

    public static final double GEAR_RATIO = 2.0;
    public static final double MOTOR_STALL_TORQUE = 0.71;

    public static final double ELBOW_P = 0.001;
	public static final double ELBOW_I = 0.4;
    public static final double ELBOW_D = 0;
    public static final double EXTENDER_P = 0.001;
    public static final double EXTENDER_I = 0;
    public static final double EXTENDER_D = 0;
}
