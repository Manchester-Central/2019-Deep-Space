/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.PIDController;
import frc.ChaosSensors.ChaosBetterTalonSRX;
import frc.ChaosSensors.TalonSRX_Encoder;

/**
 * Add your docs here.
 */
public class Wrist {

   ChaosBetterTalonSRX speedController;
   TalonSRX_Encoder speedControllerEncoder;
   PIDController pid;

   public static final double CIRCUMFERENCE = 2 * Math.PI;
   public static final double ENCODER_TICKS_PER_REVOLUTION = 4100;
   public static final double MAX_ANGLE = 180;
   public static final double MIN_ANGLE = 0;
   public static final double DEFAULT_ANGLE = 0;
   public static final double P = 0.001;
   public static final double I = 0;
   public static final double D = 0;


   public Wrist() {

      speedController = new ChaosBetterTalonSRX(PortConstants.WRIST, CIRCUMFERENCE, ENCODER_TICKS_PER_REVOLUTION,
            false);
         
      speedControllerEncoder = new TalonSRX_Encoder(speedController);
      pid = new PIDController(P, I, D, speedControllerEncoder, speedController);
   }

   public void setSpeed(double speed) {
      if ((getAngle() >= MAX_ANGLE) && (speed > 0)) {
         speed = 0;
      } else if ((getAngle() <= MIN_ANGLE) && (speed < 0)) {
         speed = 0;
      }
      speedController.set(speed);
   }

   public double getRawTicks() {
      return speedController.getCurrentPositionTicks();
   }

   public double getAngle() {
      return speedController.getEncoderAngle();
   }

   public void setSetPoint(double targetAngle) {
      pid.setSetpoint(targetAngle);
   }

   public void goToSetPoint() {
      pid.enable();
   }

   public void stopWristPID () {
      pid.disable();
   }


}
