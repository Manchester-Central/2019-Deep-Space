/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;

/**
 * Add your docs here.
 */
public class Grabber {
   
   Spark spark;
   DoubleSolenoid hatch;
   
   public Grabber () {
     
    spark = new Spark(PortConstants.GRABBER_SPARK);
    hatch = new DoubleSolenoid(PortConstants.FORWARD_HATCH, PortConstants.REVERSE_HATCH);

   }

   public void setSpark(double speed) {

    spark.set(speed);

   }

}
