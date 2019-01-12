/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Spark;

/**
 * Add your docs here.
 */
public class DriveBase {

    public Spark leftFront;
    public Spark rightFront;


    public DriveBase() {

        leftFront = new Spark(PortConstants.LEFT_FRONT_SPARK);
        rightFront = new Spark(PortConstants.RIGHT_FRONT_SPARK);

    }






}
