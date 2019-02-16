/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.ChaosSensors.AnglePot;

/**
 * Add your docs here.
 */
public class Wrist {
   
   WPI_TalonSRX speedController;
   AnglePot wristPot;
   
   public Wrist () {
     
      speedController = new WPI_TalonSRX(PortConstants.WRIST);

   }

   public void setSpeed(double speed) {

    speedController.set(speed);

   }

   public double getRawPot() {
      return wristPot.get();
   }

   public double getAngle() {
      return wristPot.getValue();
   }



}
