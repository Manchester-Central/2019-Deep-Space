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

/**
 * Add your docs here.
 */
public class Camera {

    static String entryName = "test";

    static double robotHeight = 60D;
    static double visionTargetHeight = 80D;
    

    public static NetworkTable getTable () {

        return NetworkTableInstance.getDefault().getTable("limelight");
        
    }

    public static NetworkTableEntry getEntry (String name) {

        return getTable().getEntry(name);

    }





}
