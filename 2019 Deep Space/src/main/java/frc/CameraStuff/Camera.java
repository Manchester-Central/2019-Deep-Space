/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.CameraStuff;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.FunctionsThatShouldBeInTheJDK;

/**
 * Add your docs here.
 */
public class Camera {

    static String entryName = "test";

    static double robotHeight = 60D;
    static double visionTargetHeight = 80D;
    static double cameraAngle = 30D;
    static double maxAcceleration = 0.04;
    static double maxSpeedDistance = 69; 
    

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
    
    public static double[] getDriveDirections(double[] currentSpeeds) {

        double[] driveValues = {0, 0};
        double maxNewSpeed = currentSpeeds[0] + maxAcceleration;

        driveValues[0] = (getDistance() / maxSpeedDistance) - (GetHorizontalAngle() / 27D);
        driveValues[1] = (getDistance() / maxSpeedDistance) + (GetHorizontalAngle() / 27D);

        driveValues[0] = FunctionsThatShouldBeInTheJDK.clamp(driveValues[0], -1D, 1D);
        driveValues[1] = FunctionsThatShouldBeInTheJDK.clamp(driveValues[1], -1D, 1D);

        driveValues[0] = (driveValues[0] > maxNewSpeed) ? maxNewSpeed : driveValues[0]; 
        driveValues[1] = (driveValues[1] > maxNewSpeed) ? maxNewSpeed : driveValues[1];

        return driveValues;

    }
    

}
