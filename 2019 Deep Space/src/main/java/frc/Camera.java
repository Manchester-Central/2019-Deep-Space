/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.FunctionsThatShouldBeInTheJDK;

/**
 * Add your docs here.
 */
public class Camera {

    //

    static double robotHeight = 0D;
    static double visionTargetHeight = 1D;
    static double cameraAngle = 0D;
    static double maxAcceleration = 0.04;
    static double maxSpeedDistance = 69; 
    /***
     *  states for the camera image
     */
    public static enum camState {driver, image};
    /***
     *  states for the state of the camera's led
     */
    public static enum ledState {on, off, blink, pipeline};
    

    /***
     * returns the Camera network table
     * @return Camera NetworkTable
     */
    public static NetworkTable getTable () {

        return NetworkTableInstance.getDefault().getTable("limelight");
        
    }

    /***
     * get an entry from the camera network table, access with getDouble()
     * @param name the name of the limelight variable
     * @return the NetworkTableEntry value from the table
     */
    public static NetworkTableEntry getEntry (String name) {

        return getTable().getEntry(name);

    }

    /***
     * Gets the distance from the camera to the vision target using their heights, the angle of the camera, 
     * and the read ty vartical-angle-from-center-to-target value from the limelight
     * @return the horizontal distance from camera to vision target
     */
    public static double getDistance () {

        return (visionTargetHeight - robotHeight) / (Math.tan( Math.toRadians(getEntry("ty").getDouble(69) + cameraAngle)));

    }

    /***
     * tx: horizontal angle from the center of the camera to the vision target
     * @return angle in degrees
     */
    public static double GetHorizontalAngle () {

        return getEntry("tx").getDouble(0);

    }
    
    /***
     * get the drive values for following the vision target
     * @param currentSpeedLeft current left side set speed
     * @param currentSpeedRight current right side set speed
     * @return two-value area for leftSpeed and rightSpeed
     */
    public static double[] getDriveDirections(double currentSpeedLeft, double currentSpeedRight) {


        double[] driveValues = {0D, 0D};
        //double maxNewSpeedLeft = currentSpeedLeft + maxAcceleration;
        //double maxNewSpeedRight = currentSpeedRight + maxAcceleration;

        driveValues[0] = (getDistance() / maxSpeedDistance) - (GetHorizontalAngle() / 27D);
        driveValues[1] = (getDistance() / maxSpeedDistance) + (GetHorizontalAngle() / 27D);

        driveValues[0] = FunctionsThatShouldBeInTheJDK.clamp(driveValues[0], -1D, 1D);
        driveValues[1] = FunctionsThatShouldBeInTheJDK.clamp(driveValues[1], -1D, 1D);

        //driveValues[0] = (driveValues[0] > maxNewSpeedLeft) ? maxNewSpeedLeft : driveValues[0]; 
        //driveValues[1] = (driveValues[1] > maxNewSpeedRight) ? maxNewSpeedRight : driveValues[1];

        return driveValues;

    }

    //public static 
    
    /***
     *  change the state of the camera to vision or image processing
     * @param set driver or image
     */
    public static void changeCamMode(camState set) {

        if (set.equals(camState.image)) {
            getEntry("camMode").setNumber(0);
        } else  {
            getEntry("camMode").setNumber(1); 
        } 
    
    }

    /**
     * sets the state for the camera leds
     * @param set on, off, blink, pipeline
     */
    public static void lightsOn(ledState set) {
       

        if (set.equals(ledState.on)) {
            getEntry("ledMode").setNumber(3);
        } else if (set.equals(ledState.blink)) {
            getEntry("ledMode").setNumber(2); 
        } else if (set.equals(ledState.off)){
            getEntry("ledMode").setNumber(1);
        } else {
            getEntry("ledMode").setNumber(0);
        }
    }
}
