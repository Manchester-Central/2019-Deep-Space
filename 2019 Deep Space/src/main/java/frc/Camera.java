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

    static double robotHeight = 60D;
    static double visionTargetHeight = 80D;
    static double cameraAngle = 30D;
    static double maxAcceleration = 0.04;
    static double maxSpeedDistance = 69; 
    public static enum camState {driver, image};
    public static enum ledState {on, off, blink, pipeline};
    

    public static NetworkTable getTable () {

        return NetworkTableInstance.getDefault().getTable("limelight");
        
    }

    public static NetworkTableEntry getEntry (String name) {

        return getTable().getEntry(name);

    }

    public static double getDistance () {

        return (visionTargetHeight - robotHeight) / (Math.tan(getEntry("ty").getDouble(69) + cameraAngle));

    }

    public static double GetHorizontalAngle () {

        return getEntry("tx").getDouble(0);

    }

    //public static double getDistanceFromArea () {

        // ta = 

    //}
    
    public static double[] getDriveDirections(double currentSpeedLeft, double currentSpeedRight) {


        double[] driveValues = {0D, 0D};
        double maxNewSpeedLeft = currentSpeedLeft + maxAcceleration;
        double maxNewSpeedRight = currentSpeedRight + maxAcceleration;

        driveValues[0] = (getDistance() / maxSpeedDistance) - (GetHorizontalAngle() / 27D);
        driveValues[1] = (getDistance() / maxSpeedDistance) + (GetHorizontalAngle() / 27D);

        driveValues[0] = FunctionsThatShouldBeInTheJDK.clamp(driveValues[0], -1D, 1D);
        driveValues[1] = FunctionsThatShouldBeInTheJDK.clamp(driveValues[1], -1D, 1D);

        driveValues[0] = (driveValues[0] > maxNewSpeedLeft) ? maxNewSpeedLeft : driveValues[0]; 
        driveValues[1] = (driveValues[1] > maxNewSpeedRight) ? maxNewSpeedRight : driveValues[1];

        return driveValues;

    }
    
    public static void changeCamMode(camState set) {

        if (set.equals(camState.image)) {
            getEntry("camMode").setNumber(0);
        } else  {
            getEntry("camMode").setNumber(1); 
        } 
    
    }

    public static void lightsOn(ledState set) {
       

        if (set.equals(ledState.on)) {
            getEntry("ledMode").setNumber(3);
        } else if (set.equals(ledState.blink)) {
            getEntry("camMode").setNumber(2); 
        } else if (set.equals(ledState.off)){
            getEntry("camMode").setNumber(1);
        } else {
            getEntry("camMode").setNumber(0);
        }
    }
}
