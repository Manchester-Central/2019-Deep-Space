/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.chaos;

/**
 * CHAOS Commands that don't exist, but now exist
 */
public class FunctionsThatShouldBeInTheJDK {

    public static double clamp (double value, double min, double max) {
        return Math.max((Math.min(max, value)), min);
    }

    public static double getSign(double value) {
        if (value == 0)
            return 0;
        return Math.abs(value) / value; 
    }

    public static int getSign(int value) {
        if (value == 0)
            return 0;
        return Math.abs(value) / value; 
    }

    /***
     * 
     * @param currentValue
     * @param targetValue
     * @param deadBand - The tolerance, both above and below the target
     * @return - True if the value is within a certain range above or below a target
     */
    public static boolean withinPlusOrMinus(double currentValue, double targetValue, double deadBand) {

        return Math.abs(currentValue - targetValue) < Math.abs(deadBand);

    }

    /**
     * 
     * @param value 
     * @param lowerBound 
     * @param upperBound
     * @return true if the given value is within a given range
     */
    public static boolean withinRange(double value, double lowerBound, double upperBound) {
        return (value >= lowerBound) && (value <= upperBound);
    }
    
}
