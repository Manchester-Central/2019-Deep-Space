/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Add your docs here.
 */
public class Grabber {
   
   Spark intake;
   DoubleSolenoid hatch;

   DigitalInput switchLeft;
   DigitalInput switchRight;
   DigitalInput beamSensor;

   // positive = input
   public static final double INTAKE_OUTPUT_SPEED = 0.7;

   
   public Grabber () {
     
    intake = new Spark(PortConstants.GRABBER_SPARK);
    hatch = new DoubleSolenoid(PortConstants.REVERSE_HATCH, PortConstants.FORWARD_HATCH);
    switchLeft = new DigitalInput(PortConstants.LIMIT_SWITCH_LEFT);
    switchRight = new DigitalInput(PortConstants.LIMIT_SWITCH_RIGHT);
    beamSensor = new DigitalInput (PortConstants.BEAM_SENSOR);

   }

   public void setSpark(double speed) {
      intake.set(speed);
   }

   public void extendHatchGrabber () {
      hatch.set(Value.kForward);
   }

   public void retractHatchGrabber () {
      hatch.set(Value.kReverse);
   }

   public boolean getLimitSwitchLeft () {
      return switchLeft.get();
   }

   public boolean getLimitSwitchRight () {
      return switchRight.get();
   }

   public boolean getBeamSensor () {
      return beamSensor.get();
   }
   

}
